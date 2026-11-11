"""Phase 2d - transient lap simulation of the SMA brake-duct actuator (reduced-order, calibrated to Fluent + MAPDL).

A transient Fluent CHT run over a full lap is not affordable here (~1600 time steps on a 0.5M-cell mesh with
DO radiation). Instead the steady Fluent Case A / Case B solutions and the MAPDL actuation sweep are used to
calibrate a lumped transient model:

1. Lap: synthetic F1-style speed trace (5 braking zones), repeated for 3 laps.
2. Rotor (front-left C/C disc, 1.2 kg, cp 800): lumped energy balance
       m c dTr/dt = f_brake * P_decel(t) - hA(V) (Tr - Tamb) - eps_r sigma A_r (Tr^4 - Tamb^4)
   with f_brake and hA calibrated so the periodic lap peaks at 1073 K (Case A) and the longest straight
   cools the disc to 673 K (Case B), the two rotor temperatures in the brief.
3. SMA spring position (gap from rotor face, default 6 mm): incident radiation G = a sigma Tr^4 + b and air
   speed U = c + d V fitted to the Fluent pocket planes for Case A and Case B. Air at that gap is ~300 K in
   both solves.
4. Spring wire (Nitinol, d = 1.2 mm, rho 6450, cp 837, transformation enthalpy 20 J/g): lumped transient
       rho cp_eff (d/4) dT/dt = eps G/4 - eps sigma T^4 - h(U) (T - T_air)
   cp_eff = cp + dH |dxi/dT| on the active branch (latent heat of the M<->A transformation).
   Martensite fraction xi follows Liang-Rogers kinetics with reversal points (partial loops).
5. Louver: rod stroke = f(xi) from the MAPDL heating sweep; louver angle from the crank kinematics.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

ROOT = PROJECT_ROOT
SIGMA = 5.670374419e-8
T_AMB = 300.0
GAP_MM = float(os.environ.get("GAP_MM", "6"))
EPS_W = float(os.environ.get("EPS", "0.6"))
DT = 0.01
N_LAPS = 3

# ---------------- car / lap ----------------
M_CAR, G0 = 800.0, 9.81
V_HI, V_LO = 300 / 3.6, 100 / 3.6
STRAIGHTS_S = [12.0, 6.0, 15.0, 4.0, 9.0]          # full-throttle time at 300 km/h before each braking zone
T_BRAKE, T_CORNER, T_ACCEL = 1.5, 3.0, 6.0


def lap_trace():
    t, v, braking = [], [], []
    now = 0.0

    def seg(dur, f, brk):
        nonlocal now
        n = int(round(dur / DT))
        for i in range(n):
            t.append(now + i * DT)
            v.append(f(i / n))
            braking.append(brk)
        now += n * DT

    for s in STRAIGHTS_S:
        seg(s, lambda x: V_HI, False)
        seg(T_BRAKE, lambda x: V_HI + (V_LO - V_HI) * x, True)
        seg(T_CORNER, lambda x: V_LO, False)
        seg(T_ACCEL, lambda x: V_LO + (V_HI - V_LO) * x, False)
    return np.array(t), np.array(v), np.array(braking)


t1, v1, b1 = lap_trace()
LAP_T = t1[-1] + DT
t = np.concatenate([t1 + k * LAP_T for k in range(N_LAPS)])
v = np.tile(v1, N_LAPS)
brk = np.tile(b1, N_LAPS)
decel = np.where(brk, (V_HI - V_LO) / T_BRAKE, 0.0)
p_decel = M_CAR * decel * v                       # total deceleration power [W]

# ---------------- rotor model ----------------
M_R, C_R, EPS_R = 1.2, 800.0, 0.85
A_R = 2 * math.pi * (0.164 ** 2 - 0.105 ** 2)      # two friction faces, 210-328 mm annulus


def rotor_run(params, t_end=None):
    f_brake, ha_300 = params
    tr = np.empty_like(t)
    tr[0] = 700.0
    for i in range(1, len(t)):
        ha = ha_300 * (max(v[i - 1], 5.0) / V_HI) ** 0.8
        q = f_brake * p_decel[i - 1] - ha * (tr[i - 1] - T_AMB) - EPS_R * SIGMA * A_R * (tr[i - 1] ** 4 - T_AMB ** 4)
        tr[i] = tr[i - 1] + DT * q / (M_R * C_R)
    return tr


def last_lap(a):
    return a[t >= (N_LAPS - 1) * LAP_T]


def longest_straight_min(tr):
    tl = last_lap(t) - (N_LAPS - 1) * LAP_T
    start = sum(STRAIGHTS_S[:2]) + 2 * (T_BRAKE + T_CORNER + T_ACCEL)
    m = (tl >= start) & (tl < start + STRAIGHTS_S[2])
    return last_lap(tr)[m].min()


fit = least_squares(lambda p: [last_lap(rotor_run(p)).max() - 1073.15, longest_straight_min(rotor_run(p)) - 673.15],
                    x0=[0.15, 60.0], bounds=([0.01, 1.0], [1.0, 2000.0]))
F_BRAKE, HA_300 = fit.x
TR = rotor_run(fit.x)

# ---------------- spring environment from Fluent ----------------
planes = {c: {p["gap_mm"]: p for p in json.load(open(os.path.join(ROOT, "cfd", f"pocket_planes_{c}.json")))}
          for c in ("CaseA", "CaseB")}
pa, pb = planes["CaseA"][GAP_MM], planes["CaseB"][GAP_MM]
sa, sb = SIGMA * 1073.15 ** 4, SIGMA * 673.15 ** 4
G_A = (pa["G_W_m2"] - pb["G_W_m2"]) / (sa - sb)
G_B = pa["G_W_m2"] - G_A * sa
U_D = (pb["U_m_s"] - pa["U_m_s"]) / (V_HI - V_LO)
U_C = pa["U_m_s"] - U_D * V_LO
T_AIR = 0.5 * (pa["T_air_K"] + pb["T_air_K"])
G_t = G_A * SIGMA * TR ** 4 + G_B
U_t = np.maximum(U_C + U_D * v, 0.5)

# ---------------- SMA wire ----------------
D_W, RHO_W, CP_W, DH = float(os.environ.get("WIRE_MM", "1.2")) / 1000, 6450.0, 837.0, 20e3
AS, AF, MS, MF = (float(os.environ.get(k, d)) + 273.15 for k, d in
                  (("AS_C", "75"), ("AF_C", "85"), ("MS_C", "55"), ("MF_C", "45")))
ALLOY = os.environ.get("ALLOY", "NiTi")


def air_props(T):
    k = 0.0241 * (T / 273.15) ** 0.81
    mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)
    return k, mu / (101325 / (287.05 * T)), 0.71


def h_cyl(tf, u):
    k, nu, pr = air_props(tf)
    re = max(u, 0.05) * D_W / nu
    return (0.3 + 0.62 * re ** 0.5 * pr ** (1 / 3) / (1 + (0.4 / pr) ** (2 / 3)) ** 0.25 *
            (1 + (re / 282000) ** 0.625) ** 0.8) * k / D_W


def xi_heat(T, xi0):
    if T <= AS:
        return xi0
    if T >= AF:
        return 0.0
    return 0.5 * xi0 * (math.cos(math.pi * (T - AS) / (AF - AS)) + 1)


def xi_cool(T, xi0):
    if T >= MS:
        return xi0
    if T <= MF:
        return 1.0
    return 0.5 * (1 - xi0) * math.cos(math.pi * (T - MF) / (MS - MF)) + 0.5 * (1 + xi0)


tw = np.empty_like(t)
xi = np.empty_like(t)
tw[0], xi[0] = 305.0, 1.0
branch, xi_rev = "cool", 1.0                     # reversal-point bookkeeping for partial loops
for i in range(1, len(t)):
    T0 = tw[i - 1]
    q = EPS_W * G_t[i - 1] / 4 - EPS_W * SIGMA * T0 ** 4 - h_cyl(0.5 * (T0 + T_AIR), U_t[i - 1]) * (T0 - T_AIR)
    heating = q > 0
    if heating and branch == "cool":
        branch, xi_rev = "heat", xi[i - 1]
    elif not heating and branch == "heat":
        branch, xi_rev = "cool", xi[i - 1]
    f = (lambda T: xi_heat(T, xi_rev)) if branch == "heat" else (lambda T: xi_cool(T, xi_rev))
    dxdT = (f(T0 + 0.05) - f(T0 - 0.05)) / 0.1
    cp_eff = CP_W + DH * abs(dxdT)
    tw[i] = T0 + DT * q / (RHO_W * cp_eff * D_W / 4)
    new = f(tw[i])
    xi[i] = min(xi[i - 1], new) if branch == "heat" else max(xi[i - 1], new)

# ---------------- louver kinematics from MAPDL ----------------
sweep = json.load(open(os.path.join(ROOT, "fea", "phase3_results.json")))["sweep"]["heating"]
pairs = sorted({(round(xi_heat(Tc + 273.15, 1.0), 4), max(s, 0.0)) for Tc, s, ok in sweep if ok and Tc <= 85})
XI_TAB = np.array([p[0] for p in pairs])
S_TAB = np.array([p[1] for p in pairs])
stroke = np.where(xi > 0.999, 0.0, np.interp(xi, XI_TAB, S_TAB))
CR = 14.0 / (2 * math.sin(math.radians(40)))
PHI0 = -math.radians(40)
angle = np.round(np.degrees(np.arcsin(np.clip(stroke / CR + math.sin(PHI0), -1, 1)) - PHI0), 6)

# ---------------- metrics (last lap) ----------------
L = t >= (N_LAPS - 1) * LAP_T
tl = t[L] - (N_LAPS - 1) * LAP_T
vL, bL, angL, twL, trL = v[L], brk[L], angle[L], tw[L], TR[L]
events = []
cursor = 0.0
for k, s in enumerate(STRAIGHTS_S):
    b_on = cursor + s
    zone_end = b_on + T_BRAKE + T_CORNER + T_ACCEL
    nxt_straight_end = zone_end + (STRAIGHTS_S[(k + 1) % len(STRAIGHTS_S)] if k + 1 < len(STRAIGHTS_S) else 0)
    m = (tl >= b_on) & (tl < zone_end)
    ang_zone = angL[m]
    t_zone = tl[m] - b_on
    above = np.where(ang_zone >= 40)[0]
    m_str = (tl >= zone_end) & (tl < nxt_straight_end) if k + 1 < len(STRAIGHTS_S) else np.zeros_like(tl, bool)
    events.append(dict(zone=k + 1, straight_before_s=s, max_angle_deg=float(ang_zone.max()),
                       peak_wire_K=float(twL[m].max()), peak_rotor_K=float(trL[m].max()),
                       time_to_40deg_s=float(t_zone[above[0]]) if len(above) else None,
                       mean_angle_on_next_straight_deg=float(angL[m_str].mean()) if m_str.any() else None))
    cursor = zone_end
open_on_straights = float(np.mean(angL[(vL >= V_HI - 0.1) & ~bL] >= 40) * 100)
open_in_zones = float(np.mean(angL[(vL < V_HI - 0.1) | bL] >= 40) * 100)
summary = dict(alloy=ALLOY, wire_d_mm=D_W * 1000, transformation_C=dict(As=AS - 273.15, Af=AF - 273.15, Ms=MS - 273.15, Mf=MF - 273.15),
               gap_mm=GAP_MM, wire_emissivity=EPS_W, lap_time_s=LAP_T, rotor_fit=dict(f_brake=F_BRAKE, hA_300_W_K=HA_300,
               last_lap_max_K=float(trL.max()), last_lap_min_K=float(trL.min())),
               spring_env=dict(G_fit=f"G = {G_A:.3f} sigma Tr^4 + {G_B:.0f}", U_fit=f"U = {U_C:.2f} + {U_D:.3f} V", T_air_K=T_AIR),
               wire_last_lap=dict(min_K=float(twL.min()), max_K=float(twL.max())),
               tau_wire_s_at_300kmh=RHO_W * CP_W * D_W / 4 / h_cyl(310, U_C + U_D * V_HI),
               braking_events=events, pct_time_open_on_straights=open_on_straights,
               pct_time_open_braking_corners=open_in_zones)

tag = f"{ALLOY}_Af{AF - 273.15:g}_gap{GAP_MM:g}_eps{EPS_W:g}_d{D_W * 1000:g}"
os.makedirs(os.path.join(ROOT, "cfd", "transient"), exist_ok=True)
json.dump(summary, open(os.path.join(ROOT, "cfd", "transient", f"lap_{tag}.json"), "w"), indent=2)

fig, ax = plt.subplots(4, 1, figsize=(13, 11), dpi=150, sharex=True)
ax[0].plot(t, v * 3.6, "k")
ax[0].set_ylabel("Speed [km/h]")
for a in ax:
    for i0 in np.where(np.diff(brk.astype(int)) == 1)[0]:
        a.axvspan(t[i0], t[i0] + T_BRAKE, color="crimson", alpha=0.12, lw=0)
ax[1].plot(t, TR - 273.15, color="tab:red")
ax[1].set_ylabel("Rotor [C]")
ax[2].plot(t, tw - 273.15, color="tab:orange", label="SMA wire")
for val, lab, col in ((AF, "Af", "crimson"), (AS, "As", "salmon"), (MS, "Ms", "steelblue"), (MF, "Mf", "navy")):
    ax[2].axhline(val - 273.15, color=col, ls=":", lw=1.2)
    ax[2].text(t[-1], val - 273.15, " " + lab, color=col, va="center", fontsize=8)
ax[2].set_ylabel("Wire [C]")
ax[3].plot(t, angle, color="tab:blue")
ax[3].set_ylabel("Louver angle [deg]")
ax[3].set_xlabel("Time [s]  (shaded = braking)")
ax[3].set_ylim(-5, 95)
ax[0].set_title(f"Transient lap model ({ALLOY}): SMA spring {GAP_MM:g} mm from rotor, wire emissivity {EPS_W:g} "
                f"(open on straights {open_on_straights:.0f} % of time, in braking/corners {open_in_zones:.0f} %)")
for a in ax:
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(ROOT, "cfd", "transient", f"Transient_Lap_{tag}.png"))
print(json.dumps(summary, indent=1))
