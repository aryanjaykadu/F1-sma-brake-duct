"""Mesh-independence analysis of the isolated-wheel drag (coarse / medium / fine), with the Grid Convergence
Index of Celik et al. (2008, J. Fluids Eng. 130:078001) on total drag and on the open-vs-closed drag change.
Iterative noise (half-band of the last 200 iterations) is reported next to the mesh error.
"""
from paths import ROOT as PROJECT_ROOT
import json
import math
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

D = PROJECT_ROOT + r"\cfd\wheel"
LEVELS = [("fine", ""), ("medium", "_medium"), ("coarse", "_coarse")]
V, RHO = 300 / 3.6, 1.225


def hist(v, tag):
    rows = [ln.split() for ln in open(os.path.join(D, f"drag_history_{v}{tag}.out")).read().splitlines()[3:]]
    x = np.array([float(r[1]) for r in rows if len(r) >= 3])[-200:]
    return float(x.mean()), float((x.max() - x.min()) / 2)


def cells(v, tag):
    return int(re.findall(r"(\d+) cells were created", open(os.path.join(D, f"mesh_{v}{tag}.log"), errors="replace").read())[-1])


def gci(phi, h):
    """phi, h ordered fine, medium, coarse. Returns dict with order, extrapolated value and fine-grid GCI."""
    (p1, p2, p3), (h1, h2, h3) = phi, h
    r21, r32 = h2 / h1, h3 / h2
    e21, e32 = p2 - p1, p3 - p2
    out = {"r21": r21, "r32": r32, "eps21": e21, "eps32": e32}
    if abs(e21) < 1e-12 or abs(e32) < 1e-12:
        out["note"] = "zero change between levels"
        return out
    s = math.copysign(1.0, e32 / e21)
    out["convergence"] = "monotonic" if s > 0 else "oscillatory"
    p = 2.0
    for _ in range(200):
        q = math.log((r21 ** p - s) / (r32 ** p - s)) if (r21 ** p - s) > 0 and (r32 ** p - s) > 0 else 0.0
        p_new = abs(math.log(abs(e32 / e21)) + q) / math.log(r21)
        if abs(p_new - p) < 1e-8:
            break
        p = p_new
    p = min(max(p, 0.5), 4.0)          # keep the apparent order physical for 2nd-order schemes
    ext = (r21 ** p * p1 - p2) / (r21 ** p - 1)
    ea21 = abs((p1 - p2) / p1) if p1 else float("nan")
    out.update(order=p, extrapolated=ext, ea21_pct=100 * ea21, gci_fine_pct=100 * 1.25 * ea21 / (r21 ** p - 1))
    return out


rows = []
for name, tag in LEVELS:
    r = {"level": name}
    for v in ("Open", "Closed"):
        a = json.load(open(os.path.join(D, f"aero_{v}{tag}.json")))
        m, band = hist(v, tag)
        r[v] = {"cells": cells(v, tag), "drag_N": m, "band_N": band, "throat_kg_s": a["throat_mass_flow_kg_s"],
                "duct_N": a["drag_duct_N"], "wheel_N": a["drag_wheel_N"]}
    r["cells_mean"] = 0.5 * (r["Open"]["cells"] + r["Closed"]["cells"])
    r["h"] = r["cells_mean"] ** (-1 / 3)
    r["delta_N"] = r["Closed"]["drag_N"] - r["Open"]["drag_N"]
    r["delta_pct"] = 100 * r["delta_N"] / r["Open"]["drag_N"]
    r["noise_pct"] = 100 * math.hypot(r["Open"]["band_N"], r["Closed"]["band_N"]) / r["Open"]["drag_N"]
    r["throat_change_pct"] = 100 * (r["Closed"]["throat_kg_s"] / r["Open"]["throat_kg_s"] - 1)
    r["cd_open"] = r["Open"]["drag_N"] / (0.5 * RHO * V ** 2 * 0.720 * 0.295)
    # momentum bound: a duct cannot save more drag than the momentum flux of the air it swallows
    r["bound_N"] = r["Open"]["throat_kg_s"] * V
    r["bound_pct"] = 100 * r["bound_N"] / r["Open"]["drag_N"]
    r["duct_drag_change_N"] = r["Closed"]["duct_N"] - r["Open"]["duct_N"]
    r["wheel_drag_change_N"] = r["Closed"]["wheel_N"] - r["Open"]["wheel_N"]
    rows.append(r)

h = [r["h"] for r in rows]
res = {"levels": rows,
       "gci_drag_open": gci([r["Open"]["drag_N"] for r in rows], h),
       "gci_drag_closed": gci([r["Closed"]["drag_N"] for r in rows], h),
       "gci_delta_pct": gci([r["delta_pct"] for r in rows], h),
       "gci_throat_change_pct": gci([r["throat_change_pct"] for r in rows], h)}
json.dump(res, open(os.path.join(D, "wheel_mesh_independence.json"), "w"), indent=2)

fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
n = [r["cells_mean"] / 1e3 for r in rows]
for v, c in (("Open", "tab:green"), ("Closed", "tab:red")):
    ax[0].errorbar(n, [r[v]["drag_N"] for r in rows], yerr=[r[v]["band_N"] for r in rows], fmt="o-", color=c,
                   capsize=4, label=f"Louvers {v.lower()}")
ax[0].set_xlabel("Cells [thousand]")
ax[0].set_ylabel("Total drag [N] (mean of last 200 iterations)")
ax[0].set_title("Drag vs mesh size (bars: iteration oscillation)")
ax[1].errorbar(n, [r["delta_pct"] for r in rows], yerr=[r["noise_pct"] for r in rows], fmt="o-", color="k", capsize=4,
               label="CFD difference (bars: solver noise)")
ax[1].fill_between(n, [-r["bound_pct"] for r in rows], [r["bound_pct"] for r in rows], color="tab:green", alpha=0.25,
                   label="Physical limit: duct momentum flux")
ax[1].axhline(-18.4, color="tab:red", ls="--", lw=1)
ax[1].text(n[-1], -18.4, " brief target -18.4 %", color="tab:red", va="bottom", fontsize=9)
ax[1].axhline(0, color="0.6", lw=0.8)
ax[1].set_xlabel("Cells [thousand]")
ax[1].set_ylabel("Drag change on closing [%]")
ax[1].set_title("Drag change on closing: within solver noise at every\nmesh; duct momentum drag 1.1-1.4 %")
ax[1].legend(loc="upper right", fontsize=8)
ax[2].plot(n, [r["throat_change_pct"] for r in rows], "o-", color="tab:blue")
ax[2].set_xlabel("Cells [thousand]")
ax[2].set_ylabel("Throat mass-flow change on closing [%]")
ax[2].set_title("Duct flow change")
for a in ax:
    a.grid(alpha=0.3)
ax[0].legend()
fig.tight_layout()
fig.savefig(os.path.join(D, "Wheel_Mesh_Independence.png"), dpi=150)
print(json.dumps({r["level"]: {"cells": int(r["cells_mean"]), "open_N": round(r["Open"]["drag_N"], 1),
                               "closed_N": round(r["Closed"]["drag_N"], 1), "delta_pct": round(r["delta_pct"], 2),
                               "noise_pct": round(r["noise_pct"], 2), "throat_pct": round(r["throat_change_pct"], 1),
                               "cd_open": round(r["cd_open"], 3), "bound_pct": round(r["bound_pct"], 2),
                               "duct_dN": round(r["duct_drag_change_N"], 1),
                               "wheel_dN": round(r["wheel_drag_change_N"], 1)} for r in rows}, indent=1))
for k in ("gci_drag_open", "gci_drag_closed", "gci_delta_pct", "gci_throat_change_pct"):
    print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in res[k].items()})
