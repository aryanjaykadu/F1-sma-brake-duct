"""EV re-target - compare shutter-open vs shutter-closed CFD and calibrate the drive-cycle model.

kappa = (drag open - drag closed) / (radiator mass flow open x V): the share of the radiator's momentum limit the car
actually pays as cooling drag. Convergence: drag history mean and half-band over the last 200 iterations.
Writes ev/cfd/ev_aero_results.json and ev/EV_Aero_Open_vs_Closed.png, then reruns ev_system.py with this kappa.
"""
import json
import os
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "cfd")
RHO, A_FRONT = 1.225, 2.30


def hist(tag):
    # Fluent appends _1_1, _2_1 ... when a report file already exists (e.g. from smoke tests): use the longest
    import glob
    files = glob.glob(os.path.join(D, f"drag_history_{tag}*.out"))
    path = max(files, key=lambda f: sum(1 for _ in open(f)))
    rows = [ln.split() for ln in open(path).read().splitlines()[3:]]
    x = np.array([float(r[1]) for r in rows if len(r) >= 2])[-200:]
    return float(x.mean()), float((x.max() - x.min()) / 2)


res = {"speeds": {}}
for sp in (100, 130):
    o = json.load(open(os.path.join(D, f"ev_aero_Open_{sp}.json")))
    c = json.load(open(os.path.join(D, f"ev_aero_Closed_{sp}.json")))
    v = sp / 3.6
    q = 0.5 * RHO * v ** 2
    ho, hc = hist(f"Open_{sp}"), hist(f"Closed_{sp}")
    d_open, d_closed = o["drag_total_N"], c["drag_total_N"]
    saving = d_open - d_closed
    res["speeds"][sp] = {
        "drag_open_N": d_open, "drag_closed_N": d_closed, "saving_N": saving,
        "saving_pct": 100 * saving / d_open, "Cd_open": d_open / (q * A_FRONT), "Cd_closed": d_closed / (q * A_FRONT),
        "delta_Cd": saving / (q * A_FRONT), "mdot_open_kg_s": o["radiator_mdot_kg_s"],
        "mdot_closed_kg_s": c["radiator_mdot_kg_s"], "momentum_limit_N": o["radiator_mdot_kg_s"] * v,
        "kappa": saving / (o["radiator_mdot_kg_s"] * v),
        "radiator_force_open_N": 2 * o["radiator_force_half_N"],
        "wall_drag_noise_half_band_N": {"open": 2 * ho[1], "closed": 2 * hc[1]},
    }
r100, r130 = res["speeds"][100], res["speeds"][130]
res["saving_scaling_130_over_100"] = r130["saving_N"] / r100["saving_N"]
res["v2_ratio"] = (130 / 100) ** 2
res["kappa_mean"] = 0.5 * (r100["kappa"] + r130["kappa"])
json.dump(res, open(os.path.join(D, "ev_aero_results.json"), "w"), indent=2)

fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
x = np.arange(2)
ax[0].bar(x - 0.18, [r100["drag_open_N"], r130["drag_open_N"]], 0.36, color="tab:green", label="Shutters open")
ax[0].bar(x + 0.18, [r100["drag_closed_N"], r130["drag_closed_N"]], 0.36, color="tab:blue", label="Shutters closed")
ax[0].set_xticks(x, ["100 km/h", "130 km/h"])
ax[0].set_ylabel("Car drag [N] (full car)")
ax[0].set_title(f"Closing the shutters saves {r100['saving_pct']:.1f} % drag (ΔCd {r100['delta_Cd']:.3f})")
ax[0].legend()
ax[0].grid(alpha=0.3, axis="y")
ax[1].bar(["Momentum limit\n(m_dot x V)", "Actual saving\n(CFD)"], [r100["momentum_limit_N"], r100["saving_N"]],
          color=["0.6", "tab:red"])
ax[1].set_ylabel("Force at 100 km/h [N]")
ax[1].set_title(f"Cooling drag paid: {100 * r100['kappa']:.0f} % of the radiator's momentum limit")
ax[1].grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "EV_Aero_Open_vs_Closed.png"), dpi=150)
print(json.dumps(res, indent=1))
subprocess.run([sys.executable, os.path.join(HERE, "ev_system.py")], cwd=HERE,
               env=dict(os.environ, KAPPA=f"{res['kappa_mean']:.4f}"))
