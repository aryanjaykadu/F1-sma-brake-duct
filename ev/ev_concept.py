"""EV re-target - concept sizing for a passive SMA grille shutter on the low-temperature (battery / e-drive) radiator.

Computes, for a compact EV:
  * radiator through-flow and the momentum ("cooling drag") limit vs speed, to compare with the F1 brake duct
  * louver aerodynamic hinge torque vs speed and the SMA spring pack needed (with margin)
  * SMA alloy window and the coolant-temperature behaviour it gives
Writes ev/ev_concept.json and ev/EV_Concept_Sizing.png.
"""
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RHO = 1.2

# ---- vehicle and radiator (compact EV, representative values) ----
VEH = dict(mass_kg=1800, frontal_area_m2=2.30, cd_shutters_closed=0.27, crr=0.009)
RAD = dict(width_m=0.45, height_m=0.35, depth_m=0.025, loss_coeff_K=10.0)   # dp = K * 0.5 rho u_face^2
RAD["area_m2"] = RAD["width_m"] * RAD["height_m"]
# grille inlet + under-hood path, lumped: dp_system = K_sys * 0.5 rho u_face^2, driven by ~ C_p0 * 0.5 rho V^2
K_SYSTEM = RAD["loss_coeff_K"] + 6.0       # radiator + grille + under-hood exit losses
CP_DRIVE = 0.9                             # stagnation pressure recovered at the grille (fraction of 0.5 rho V^2)

# ---- shutter ----
SHUT = dict(n_louvers=6, louver_span_m=0.44, louver_chord_m=0.058, open_angle_deg=80.0)
# SMA: low-temperature NiTi (anti-scald valve class), coolant-line sensing
SMA = dict(As_C=35.0, Af_C=40.0, Ms_C=30.0, Mf_C=25.0)


def face_velocity(v):
    """Radiator face velocity with shutters fully open (Bernoulli balance of grille drive vs losses)."""
    return v * math.sqrt(CP_DRIVE / K_SYSTEM)


def main():
    speeds_kmh = np.arange(20, 141, 10)
    rows = []
    for vk in speeds_kmh:
        v = vk / 3.6
        uf = face_velocity(v)
        mdot = RHO * uf * RAD["area_m2"]
        bound = mdot * v                                            # momentum limit of cooling drag
        body = 0.5 * RHO * v ** 2 * VEH["cd_shutters_closed"] * VEH["frontal_area_m2"]
        # louver hinge torque: closed louver carries the full grille pressure on half a chord (worst case)
        dp_closed = CP_DRIVE * 0.5 * RHO * v ** 2
        torque_per_louver = dp_closed * SHUT["louver_span_m"] * SHUT["louver_chord_m"] ** 2 / 8
        rows.append(dict(speed_kmh=float(vk), face_velocity_m_s=uf, radiator_mdot_kg_s=mdot,
                         cooling_drag_bound_N=bound, body_drag_closed_N=body,
                         bound_pct_of_total=100 * bound / (body + bound),
                         louver_torque_Nmm=1000 * torque_per_louver))
    at = {int(r["speed_kmh"]): r for r in rows}
    # SMA pack: drive the 6-louver linkage through a crank of r = 15 mm against the worst aero torque at 130 km/h
    # plus friction (25 %) and a 1.5 safety factor
    crank_r = 0.015
    t130 = at[130]["louver_torque_Nmm"] / 1000 * SHUT["n_louvers"]
    force_needed = 1.5 * 1.25 * t130 / crank_r
    stroke = 2 * crank_r * math.sin(math.radians(SHUT["open_angle_deg"] / 2))
    # one 1.0 mm NiTi spring at 250 MPa recovery stress, 5 mm coil: F = tau * pi d^3 / (8 D). Coolant temperature
    # changes over minutes, so a thick (slow) wire is fine here, unlike the F1 brake duct.
    d, Dm, tau = 1.0e-3, 5.0e-3, 250e6
    f_one = tau * math.pi * d ** 3 / (8 * Dm)
    n_springs = math.ceil(force_needed / f_one)
    res = dict(vehicle=VEH, radiator=RAD, shutter=SHUT, sma=SMA, k_system=K_SYSTEM, cp_drive=CP_DRIVE, table=rows,
               shutter_force_needed_N=force_needed, shutter_stroke_mm=1000 * stroke, crank_r_mm=1000 * crank_r,
               sma_spring=dict(wire_mm=1.0, coil_mm=5.0, force_each_N=f_one, count=n_springs),
               comparison=dict(f1_duct_mdot_kg_s=0.155, f1_bound_pct=1.1,
                               ev_mdot_100kmh=at[100]["radiator_mdot_kg_s"],
                               ev_bound_pct_100kmh=at[100]["bound_pct_of_total"]))
    json.dump(res, open(os.path.join(HERE, "ev_concept.json"), "w"), indent=2)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
    s = [r["speed_kmh"] for r in rows]
    ax[0].plot(s, [r["body_drag_closed_N"] for r in rows], "k-", label="Car drag, shutters closed")
    ax[0].plot(s, [r["cooling_drag_bound_N"] for r in rows], "tab:red", label="Radiator cooling-drag limit (open)")
    ax[0].set_xlabel("Speed [km/h]")
    ax[0].set_ylabel("Force [N]")
    ax[0].set_title(f"EV radiator: closing the shutters can save up to\n{at[100]['bound_pct_of_total']:.0f} % of drag at "
                    f"100 km/h (F1 brake duct: 1.1 %)")
    ax[0].grid(alpha=0.3)
    ax[0].legend()
    T = np.linspace(15, 50, 300)

    def xi_h(t):
        return np.where(t <= SMA["As_C"], 1, np.where(t >= SMA["Af_C"], 0,
                        0.5 * (np.cos(np.pi * (t - SMA["As_C"]) / (SMA["Af_C"] - SMA["As_C"])) + 1)))

    def xi_c(t):
        return np.where(t >= SMA["Ms_C"], 0, np.where(t <= SMA["Mf_C"], 1,
                        0.5 * np.cos(np.pi * (t - SMA["Mf_C"]) / (SMA["Ms_C"] - SMA["Mf_C"])) + 0.5))
    ax[1].plot(T, 100 * (1 - xi_h(T)), "tab:red", label="Heating: opens")
    ax[1].plot(T, 100 * (1 - xi_c(T)), "tab:blue", label="Cooling: closes")
    ax[1].set_xlabel("Coolant temperature at the SMA sleeve [°C]")
    ax[1].set_ylabel("Shutter opening [%]")
    ax[1].set_title("Low-temperature NiTi window (As 35 / Af 40 / Ms 30 / Mf 25 °C)")
    ax[1].grid(alpha=0.3)
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "EV_Concept_Sizing.png"), dpi=150)
    print(json.dumps({k: v for k, v in res.items() if k not in ("table", "vehicle", "radiator", "shutter", "sma")},
                     indent=1, default=float))
    for r in rows:
        if int(r["speed_kmh"]) in (50, 100, 130):
            print({k: round(v, 2) for k, v in r.items()})


if __name__ == "__main__":
    main()
