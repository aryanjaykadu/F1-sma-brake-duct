"""Phase 2i - compare isolated-wheel drag, louvers closed vs open (reads aero_*.json and drag_history_*.out)."""
from paths import ROOT as PROJECT_ROOT
import json
import os

D = PROJECT_ROOT + r"\cfd\wheel"
V, RHO = 300 / 3.6, 1.225


def hist(v):
    rows = [ln.split() for ln in open(os.path.join(D, f"drag_history_{v}.out")).read().splitlines()[3:]]
    return [float(r[1]) for r in rows if len(r) >= 3]


r = {v: json.load(open(os.path.join(D, f"aero_{v}.json"))) for v in ("Open", "Closed")}
h = {v: hist(v)[-200:] for v in r}
avg = {v: sum(x) / len(x) for v, x in h.items()}
band = {v: (max(x) - min(x)) / 2 for v, x in h.items()}
out = {
    "mesh_cells": {"Closed": 765201, "Open": 769990},
    "final": {v: {k: r[v][k] for k in r[v] if k.endswith(("_N", "_kg_s")) or k == "Cd_assembly"} for v in r},
    "drag_total_avg_last200_N": avg,
    "drag_total_oscillation_half_band_N": band,
    "delta_closing_N": avg["Closed"] - avg["Open"],
    "delta_closing_pct": 100 * (avg["Closed"] - avg["Open"]) / avg["Open"],
    "throat_flow_change_pct": 100 * (r["Closed"]["throat_mass_flow_kg_s"] / r["Open"]["throat_mass_flow_kg_s"] - 1),
    "duct_momentum_drag_bound_N": r["Open"]["throat_mass_flow_kg_s"] * V,
    "duct_momentum_bound_pct": 100 * r["Open"]["throat_mass_flow_kg_s"] * V / avg["Open"],
    "target_pct": -18.4,
}
json.dump(out, open(os.path.join(D, "wheel_aero_open_vs_closed.json"), "w"), indent=2)
print(json.dumps(out, indent=2))
