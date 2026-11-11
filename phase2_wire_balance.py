"""SMA spring-wire temperature in the relocated pocket (rotor wake face), Case A.

Inputs come from Fluent (relocated Case A, DO radiation): area-averaged incident radiation G, air temperature
and velocity on planes 0.5-15 mm outboard of the rotor face (pocket_planes.json).

Steady balance on a gray wire (d = 1.2 mm) in a radiation field. A convex body absorbs eps * G / 4 per unit
surface area (projected-area theorem); coil self-shadowing is covered by a view-scale factor:

    view * eps * G / 4  =  eps * sigma * Tw^4  +  h * (Tw - T_air)

h from Churchill-Bernstein (cylinder in cross-flow). Conduction into the spring end fittings and the rod is
neglected, so these are upper-bound wire temperatures for a spring held in place at that gap.
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
TAG = os.environ.get('TAG', 'CaseA')

from scipy.optimize import brentq

SIGMA = 5.670374419e-8
D_WIRE = 1.2e-3
AF_K, AS_K = 358.15, 348.15
rows = json.load(open(PROJECT_ROOT + r"\cfd\pocket_planes_" + TAG + ".json"))


def air_props(T):
    k = 0.0241 * (T / 273.15) ** 0.81
    mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)
    return k, mu / (101325 / (287.05 * T)), 0.71


def h_cyl(t_film, v):
    k, nu, pr = air_props(t_film)
    re = max(v, 0.05) * D_WIRE / nu
    nu_d = 0.3 + 0.62 * re ** 0.5 * pr ** (1 / 3) / (1 + (0.4 / pr) ** (2 / 3)) ** 0.25 * \
        (1 + (re / 282000) ** 0.625) ** 0.8
    return nu_d * k / D_WIRE


def wire_T(g, ta, v, eps, view):
    f = lambda tw: view * eps * g / 4 - eps * SIGMA * tw ** 4 - h_cyl(0.5 * (tw + ta), v) * (tw - ta)  # noqa: E731
    return brentq(f, 200.0, 2000.0)


def required_eps(g, ta, v, view, target=AF_K):
    f = lambda e: wire_T(g, ta, v, e, view) - target  # noqa: E731
    if f(0.01) >= 0:
        return 0.0                 # air alone is hot enough
    return brentq(f, 0.01, 1.0) if f(1.0) > 0 else None


def lab(e):
    return ">1" if e is None else ("any" if e == 0.0 else f"{e:.2f}")


cases = []
print("gap  G[kW/m2]  Tair[K]  U[m/s] |  Tw [K] for emissivity 0.2 / 0.4 / 0.6 / 0.8 "
      "(view 1.0)  | view 0.7 @0.6 | 2xU @0.6 | eps needed for Af")
for r in rows:
    g, ta, v = r["G_W_m2"], r["T_air_K"], r["U_m_s"]
    tws = {e: wire_T(g, ta, v, e, 1.0) for e in (0.2, 0.4, 0.6, 0.8)}
    shaded = wire_T(g, ta, v, 0.6, 0.7)
    fast = wire_T(g, ta, 2 * v, 0.6, 1.0)
    need = required_eps(g, ta, v, 1.0)
    need_sh = required_eps(g, ta, v, 0.7)
    cases.append(dict(r, Tw_by_eps={str(k): w for k, w in tws.items()}, Tw_eps06_view07=shaded,
                      Tw_eps06_2xU=fast, eps_needed_view1=need, eps_needed_view07=need_sh,
                      h_W_m2K=h_cyl(0.5 * (tws[0.6] + ta), v)))
    print(f"{r['gap_mm']:4.1f}  {g / 1e3:7.1f}  {ta:7.1f}  {v:5.2f}  |  " +
          " / ".join(f"{tws[e]:5.1f}" for e in (0.2, 0.4, 0.6, 0.8)) +
          f"  |     {shaded:5.1f}     |  {fast:5.1f}  |  "
          f"{lab(need):>4s} (shaded {lab(need_sh)})")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

gaps = [c["gap_mm"] for c in cases]
fig, ax = plt.subplots(figsize=(9, 5.5), dpi=180)
for e, col in (("0.2", "tab:blue"), ("0.4", "tab:green"), ("0.6", "tab:orange"), ("0.8", "tab:red")):
    ax.plot(gaps, [c["Tw_by_eps"][e] for c in cases], "o-", color=col, label=f"wire, emissivity {e}")
ax.plot(gaps, [c["Tw_eps06_view07"] for c in cases], "--", color="tab:orange", label="emissivity 0.6, coil shading 30 %")
ax.plot(gaps, [c["T_air_K"] for c in cases], "k-", lw=2.5, label="air (Fluent)")
ax.axhline(AF_K, color="crimson", ls=":", lw=2, label="Af = 85 C (358 K)")
ax.set_xlabel("Distance of spring wire from rotor outboard face [mm]")
ax.set_ylabel("Temperature [K]")
ax.set_ylim(290, 520)
ax.axhline(318.15, color="navy", ls=":", lw=2, label="Mf = 45 C (318 K), fully closed below")
ax.set_ylim(290, 520 if TAG == "CaseA" else 420)
ax.set_title({"CaseA": "Case A (100 km/h, 800 C rotor, louvers closed)",
              "CaseB": "Case B (300 km/h, 400 C rotor, louvers open)"}.get(TAG, TAG) +
             ": SMA wire temperature with rotor radiation", fontsize=10)
ax.grid(alpha=0.3)
ax.legend(fontsize=8, loc="upper right")
fig.tight_layout()
fig.savefig(PROJECT_ROOT + r"\cfd\SMA_Wire_Temperature_" + TAG + ".png")

json.dump({"method": __doc__, "Af_K": AF_K, "wire_d_m": D_WIRE, "planes": cases},
          open(PROJECT_ROOT + r"\cfd\phase2_wire_temperature_" + TAG + ".json", "w"), indent=2)
