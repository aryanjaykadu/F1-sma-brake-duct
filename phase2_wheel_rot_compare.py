"""Time-averaged drag, rotating wheel + moving ground: louvers open vs closed, with statistical uncertainty.

Per-time-step drag from phase2_wheel_solve_rot.py. The first DISCARD steps (wash-out after switching from steady)
are dropped. The standard error of each mean uses the integrated autocorrelation time (samples are correlated):
N_eff = N / (1 + 2 sum rho_k), SE = std / sqrt(N_eff); a batch-means estimate is reported as a cross-check.
"""
from paths import ROOT as PROJECT_ROOT
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

D = PROJECT_ROOT + r"\cfd\wheel"
DISCARD = int(os.environ.get("DISCARD", "60"))
V, RHO, A_REF = 300 / 3.6, 1.225, 0.720 * 0.295
Q = 0.5 * RHO * V ** 2


def series(v):
    rows = [ln.split() for ln in open(os.path.join(D, f"drag_history_{v}_rot_transient.out")).read().splitlines()[3:]]
    a = np.array([[float(x) for x in r[:4]] for r in rows if len(r) >= 4])
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]          # step, total, duct, wheel


def stats(x):
    x = np.asarray(x, float)
    n = len(x)
    xm = x - x.mean()
    var = xm.var()
    rho_sum = 0.0
    for k in range(1, n // 2):
        rk = np.dot(xm[:-k], xm[k:]) / ((n - k) * var) if var > 0 else 0.0
        if rk <= 0:
            break
        rho_sum += rk
    neff = n / (1 + 2 * rho_sum)
    se = x.std(ddof=1) / np.sqrt(neff)
    nb = 5
    bm = np.array([b.mean() for b in np.array_split(x, nb)])
    se_batch = bm.std(ddof=1) / np.sqrt(nb)
    return {"mean": float(x.mean()), "std": float(x.std(ddof=1)), "n": n, "n_eff": float(neff),
            "se": float(se), "se_batch": float(se_batch)}


res = {"discard_steps": DISCARD}
data = {}
for v in ("Open", "Closed"):
    j = json.load(open(os.path.join(D, f"aero_{v}_rot.json")))
    step, tot, duct, wheel = series(v)
    m = step > DISCARD
    data[v] = (step, tot)
    res[v] = {"steady_total_N": j["steady_drag_total_N"], "steady_Cd": j["steady_Cd"],
              "steady_duct_N": j["steady_drag_duct_N"],
              "total": stats(tot[m]), "duct": stats(duct[m]), "wheel": stats(wheel[m]),
              "dt_s": j["dt_s"], "averaged_time_s": float(m.sum() * j["dt_s"]),
              "transient_minutes": j.get("transient_minutes")}
    res[v]["Cd_mean"] = res[v]["total"]["mean"] / (Q * A_REF)

o, c = res["Open"]["total"], res["Closed"]["total"]
se = max(np.hypot(o["se"], c["se"]), np.hypot(o["se_batch"], c["se_batch"]))
delta = c["mean"] - o["mean"]
res["delta_N"] = delta
res["delta_pct"] = 100 * delta / o["mean"]
res["delta_se_N"] = float(se)
res["delta_95ci_pct"] = [100 * (delta - 1.96 * se) / o["mean"], 100 * (delta + 1.96 * se) / o["mean"]]
res["resolved"] = bool(abs(delta) > 1.96 * se)
res["duct_delta_N"] = res["Closed"]["duct"]["mean"] - res["Open"]["duct"]["mean"]
res["steady_delta_pct"] = 100 * (res["Closed"]["steady_total_N"] / res["Open"]["steady_total_N"] - 1)
json.dump(res, open(os.path.join(D, "wheel_rotating_open_vs_closed.json"), "w"), indent=2)

fig, ax = plt.subplots(1, 2, figsize=(15, 5), gridspec_kw={"width_ratios": [2.2, 1]})
for v, col in (("Open", "tab:green"), ("Closed", "tab:red")):
    step, tot = data[v]
    t = step * res[v]["dt_s"] * 1000
    ax[0].plot(t, tot, color=col, lw=1, alpha=0.8, label=f"Louvers {v.lower()}")
    ax[0].axhline(res[v]["total"]["mean"], color=col, ls="--", lw=1)
ax[0].axvspan(0, DISCARD * res["Open"]["dt_s"] * 1000, color="0.85", alpha=0.6, label="Wash-out (discarded)")
ax[0].set_xlabel("Flow time after steady start [ms]")
ax[0].set_ylabel("Total drag [N]")
ax[0].set_title("Rotating wheel, moving ground, 300 km/h: transient drag")
ax[0].grid(alpha=0.3)
ax[0].legend()
means = [res[v]["total"]["mean"] for v in ("Open", "Closed")]
errs = [1.96 * max(res[v]["total"]["se"], res[v]["total"]["se_batch"]) for v in ("Open", "Closed")]
ax[1].bar(["Open", "Closed"], means, yerr=errs, capsize=8, color=["tab:green", "tab:red"], alpha=0.75)
lo = min(means) - 4 * max(errs) - 10
ax[1].set_ylim(lo, max(means) + 4 * max(errs) + 10)
ax[1].set_ylabel("Time-averaged drag [N] (95 % interval)")
ci = res["delta_95ci_pct"]
ax[1].set_title(f"Change on closing: {res['delta_pct']:+.2f} %\n95 % interval {ci[0]:+.2f} to {ci[1]:+.2f} % "
                f"({'resolved' if res['resolved'] else 'not resolved'})")
ax[1].grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(D, "Wheel_Rotating_Transient_Drag.png"), dpi=150)
print(json.dumps({k: res[k] for k in ("delta_N", "delta_pct", "delta_se_N", "delta_95ci_pct", "resolved",
                                       "duct_delta_N", "steady_delta_pct")}, indent=1))
for v in ("Open", "Closed"):
    r = res[v]
    print(v, f"steady {r['steady_total_N']:.1f} N (Cd {r['steady_Cd']:.3f}) | transient mean {r['total']['mean']:.1f} N "
             f"+/- {r['total']['se']:.1f} (batch {r['total']['se_batch']:.1f}), Cd {r['Cd_mean']:.3f}, "
             f"duct {r['duct']['mean']:.1f} N, averaged {r['averaged_time_s'] * 1000:.0f} ms, n_eff {r['total']['n_eff']:.1f}")
