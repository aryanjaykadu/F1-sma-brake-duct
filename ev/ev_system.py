"""EV re-target - drive-cycle model of a battery / e-drive cooling loop with a passive SMA grille shutter.

Strategies compared on each scenario:
  open        shutters always open (no shutter)
  closed      shutters always closed (only leakage air): shows the cooling need
  thermostat  motorised shutter as fitted today: open above 38 C coolant, closed below 32 C
  sma         passive SMA shutter: springs in a sleeve on the coolant return line (lag TAU_SLEEVE),
              low-temperature NiTi As 35 / Af 40 / Ms 30 / Mf 25 C, Liang-Rogers kinetics with return-point memory

Vehicle: compact EV, 1800 kg, CdA 0.62 m2 with shutters closed. Heat into the loop: motor + inverter losses
(7 % of drive power) and battery I^2 R (400 V pack, 0.08 ohm). Lumped thermal masses: battery 450 kJ/K, coolant loop
20 kJ/K, UA battery-coolant 300 W/K. Radiator: effectiveness 0.7 on the air-side capacity rate. Air mass flow =
open fraction x full-open flow (Bernoulli, ev_concept.py) + 8 % leakage when closed. Cooling drag = KAPPA x m_dot x V
(KAPPA = fraction of the momentum limit actually paid; set from CFD with env KAPPA, default 0.6 until then).
Full-open radiator flow coefficient PHI = m_dot / (rho A V) is taken from the CFD (cfd/ev_aero_results.json) when it
exists, otherwise from the Bernoulli estimate. Cooling drag is part of the road load, so it also shows in the battery
energy; savings are reported both in Wh/100 km and as a share of the always-open consumption.
Drive cycles are SYNTHETIC: "WLTC-like" uses the WLTC class-3 phase durations and top speeds, not the official trace.
Writes ev/ev_system.json and ev/EV_System_*.png.
"""
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
C = json.load(open(os.path.join(HERE, "ev_concept.json")))
RHO, CP_AIR, G = 1.2, 1005.0, 9.81
RHO_CFD = 1.225
M, CDA, CRR = 1800.0, C["vehicle"]["cd_shutters_closed"] * C["vehicle"]["frontal_area_m2"], C["vehicle"]["crr"]
A_RAD, K_SYS, CP_DRIVE = C["radiator"]["area_m2"], C["k_system"], C["cp_drive"]
KAPPA = float(os.environ.get("KAPPA", "0.6"))
_CFD = os.path.join(HERE, "cfd", "ev_aero_results.json")
if os.path.exists(_CFD):
    _sp = json.load(open(_CFD))["speeds"]
    PHI = float(np.mean([d["mdot_open_kg_s"] / (RHO_CFD * A_RAD * float(k) / 3.6) for k, d in _sp.items()]))
    PHI_SOURCE = "CFD"
else:
    PHI, PHI_SOURCE = math.sqrt(CP_DRIVE / K_SYS), "Bernoulli estimate"
LEAK = 0.08
ETA_LOSS, V_PACK, R_PACK, AUX = 0.07, 400.0, 0.08, 100.0
C_BATT, C_COOL, UA_BC, EPS_RAD = 450e3, 20e3, 300.0, 0.7
TAU_SLEEVE = 40.0
AS, AF, MS, MF = (C["sma"][k] for k in ("As_C", "Af_C", "Ms_C", "Mf_C"))
THETA_MAX = math.radians(80.0)
DT = 1.0


# ------------------------------------------------------------------ drive cycles (synthetic)
def trip_profile(trips, accel=1.0, decel=1.2):
    v = []
    for vmax_kmh, cruise_s, idle_s in trips:
        vm = vmax_kmh / 3.6
        cur = 0.0
        while cur < vm:
            cur = min(vm, cur + accel * DT)
            v.append(cur)
        v += [vm] * int(cruise_s)
        while cur > 0:
            cur = max(0.0, cur - decel * DT)
            v.append(cur)
        v += [0.0] * int(idle_s)
    return v


def wltc_like():
    phases = [  # (duration, trips) - WLTC class 3: 589 / 433 / 455 / 323 s, top 56.5 / 76.6 / 97.4 / 131.3 km/h
        (589, [(30, 20, 20), (45, 30, 15), (56.5, 45, 20), (35, 25, 20), (50, 40, 25), (25, 15, 20), (45, 30, 30)]),
        (433, [(50, 40, 10), (76.6, 60, 15), (60, 40, 10), (45, 25, 10)]),
        (455, [(70, 60, 10), (97.4, 90, 10), (80, 50, 10)]),
        (323, [(131.3, 120, 0), (110, 40, 0)]),
    ]
    out = []
    for dur, trips in phases:
        seg = trip_profile(trips)
        seg = (seg + [0.0] * dur)[:dur] if len(seg) < dur else seg[:dur]
        out += seg
    return np.array(out)


def motorway(minutes=40, vk=130):
    ramp = trip_profile([(vk, minutes * 60, 0)], accel=1.2)
    return np.array(ramp)


SCENARIOS = {
    "city_wltc_x2_25C": (np.concatenate([wltc_like(), wltc_like()]), 25.0),
    "motorway_130_30C": (motorway(40, 130), 30.0),
    "hot_day_motorway_38C": (motorway(40, 130), 38.0),
    "cold_day_wltc_x2_5C": (np.concatenate([wltc_like(), wltc_like()]), 5.0),
}


# ------------------------------------------------------------------ SMA kinetics (return-point memory)
def g_heat(T):
    if T <= AS:
        return 1.0
    if T >= AF:
        return 0.0
    return 0.5 * (math.cos(math.pi * (T - AS) / (AF - AS)) + 1)


def g_cool(T):
    if T >= MS:
        return 0.0
    if T <= MF:
        return 1.0
    return 0.5 * math.cos(math.pi * (T - MF) / (MS - MF)) + 0.5


class SMA:
    def __init__(self, T0):
        self.xi, self.branch, self.h, self.c, self.start, self.T = 1.0, "cool", 1.0, 0.0, 1.0, T0

    def step(self, T):
        heating = T > self.T
        if heating != (self.branch == "heat"):
            if abs(self.xi - self.start) > 1e-6:
                if heating:
                    g = g_heat(self.T)
                    self.h = self.xi / g if g > 1e-9 else 0.0
                else:
                    g = g_cool(self.T)
                    self.c = (1 - self.xi) / (1 - g) if g < 1 - 1e-9 else 0.0
            self.branch, self.start = ("heat" if heating else "cool"), self.xi
        new = self.h * g_heat(T) if self.branch == "heat" else 1 - self.c * (1 - g_cool(T))
        self.xi = min(self.xi, new) if self.branch == "heat" else max(self.xi, new)
        self.T = T
        return 1 - self.xi                         # stroke fraction (0 closed, 1 open)


def open_fraction_from_stroke(s):
    return math.sin(THETA_MAX * s) / math.sin(THETA_MAX)   # louver flow area vs rotation


# ------------------------------------------------------------------ simulation
def run(v, t_amb, strategy):
    n = len(v)
    tb = tc = t_amb
    tsleeve = t_amb
    sma = SMA(t_amb)
    therm_open = False
    out = {k: np.zeros(n) for k in ("tb", "tc", "open", "q_heat", "q_rad", "f_cool", "p_aero", "p_batt")}
    for i in range(n):
        vi = v[i]
        a = (v[i] - v[i - 1]) / DT if i else 0.0
        # shutter position (from the coolant state at the start of the step)
        if strategy == "open":
            of = 1.0
        elif strategy == "closed":
            of = 0.0
        elif strategy == "thermostat":
            if tc > 38.0:
                therm_open = True
            elif tc < 32.0:
                therm_open = False
            of = 1.0 if therm_open else 0.0
        else:
            tsleeve += (tc - tsleeve) * DT / TAU_SLEEVE
            of = open_fraction_from_stroke(sma.step(tsleeve))
        mdot = RHO * PHI * vi * A_RAD * (LEAK + (1 - LEAK) * of)
        f_cool = KAPPA * mdot * vi
        f_res = M * a + 0.5 * RHO * CDA * vi ** 2 + f_cool + CRR * M * G * (vi > 0.1)
        p_mech = f_res * vi
        p_batt = p_mech / 0.9 if p_mech > 0 else p_mech * 0.6
        q_heat = ETA_LOSS * abs(p_mech) + (p_batt / V_PACK) ** 2 * R_PACK + AUX
        q_rad = EPS_RAD * mdot * CP_AIR * max(tc - t_amb, 0.0)
        q_bc = UA_BC * (tb - tc)
        tb += DT * (q_heat * 0.6 - q_bc) / C_BATT                       # 60 % of the heat is generated in the pack
        tc += DT * (q_heat * 0.4 + q_bc - q_rad) / C_COOL               # 40 % (e-drive) goes straight to coolant
        for k, val in (("tb", tb), ("tc", tc), ("open", of), ("q_heat", q_heat), ("q_rad", q_rad),
                       ("f_cool", f_cool), ("p_batt", p_batt + AUX), ("p_aero", (0.5 * RHO * CDA * vi ** 2 + f_cool) * vi)):
            out[k][i] = val
    dist_km = np.sum(v) * DT / 1000
    return out, {"t_batt_max_C": float(out["tb"].max()), "t_cool_max_C": float(out["tc"].max()),
                 "time_cool_above_40C_s": float(np.sum(out["tc"] > 40) * DT),
                 "cooling_drag_energy_Wh": float(np.sum(out["f_cool"] * v) * DT / 3600),
                 "open_pct_time": float(100 * np.mean(out["open"] > 0.5)),
                 "battery_Wh_per_100km": float(np.sum(out["p_batt"]) * DT / 3600 / dist_km * 100),
                 "distance_km": float(dist_km)}


def main():
    res = {"kappa": KAPPA, "phi_full_open": PHI, "phi_source": PHI_SOURCE, "scenarios": {}}
    for name, (v, tamb) in SCENARIOS.items():
        r = {}
        traces = {}
        for strat in ("open", "closed", "thermostat", "sma"):
            tr, m = run(v, tamb, strat)
            r[strat] = m
            traces[strat] = tr
        base = r["open"]["cooling_drag_energy_Wh"]
        for strat in r:
            r[strat]["cooling_drag_saved_Wh_per_100km"] = (base - r[strat]["cooling_drag_energy_Wh"]) / r[strat][
                "distance_km"] * 100
            r[strat]["battery_saved_pct"] = 100 * (1 - r[strat]["battery_Wh_per_100km"]
                                                   / r["open"]["battery_Wh_per_100km"])
        res["scenarios"][name] = {"ambient_C": tamb, "results": r}
        fig, ax = plt.subplots(3, 1, figsize=(13, 8.5), sharex=True)
        t = np.arange(len(v)) / 60
        ax[0].plot(t, v * 3.6, "k", lw=0.8)
        ax[0].set_ylabel("Speed [km/h]")
        for strat, col in (("open", "tab:green"), ("thermostat", "tab:purple"), ("sma", "tab:red"), ("closed", "0.5")):
            ax[1].plot(t, traces[strat]["tc"], color=col, lw=1.2, label=strat)
            if strat in ("thermostat", "sma"):
                ax[2].plot(t, 100 * traces[strat]["open"], color=col, lw=1.2, label=strat)
        for y, c in ((AF, "tab:red"), (MF, "tab:blue")):
            ax[1].axhline(y, color=c, ls=":", lw=1)
        ax[1].set_ylabel("Coolant [°C]")
        ax[1].legend(loc="upper left", fontsize=8)
        ax[2].set_ylabel("Shutter open [%]")
        ax[2].set_xlabel("Time [min]")
        ax[2].legend(loc="upper left", fontsize=8)
        s = r["sma"]
        ax[0].set_title(f"{name} (ambient {tamb:g} °C, synthetic cycle): SMA shutter saves "
                        f"{s['cooling_drag_saved_Wh_per_100km']:.0f} Wh/100 km ({s['battery_saved_pct']:.1f} % of battery energy), coolant max "
                        f"{s['t_cool_max_C']:.1f} °C (always open {r['open']['t_cool_max_C']:.1f} °C)")
        for a in ax:
            a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(HERE, f"EV_System_{name}.png"), dpi=130)
        plt.close(fig)
    json.dump(res, open(os.path.join(HERE, "ev_system.json"), "w"), indent=2)
    print("KAPPA", KAPPA, "PHI", round(PHI, 4), PHI_SOURCE)
    for name, d in res["scenarios"].items():
        print(f"== {name} (ambient {d['ambient_C']} C)")
        for strat, m in d["results"].items():
            print(f"  {strat:10s} coolant max {m['t_cool_max_C']:5.1f} C, batt max {m['t_batt_max_C']:5.1f} C, "
                  f">40C {m['time_cool_above_40C_s']:6.0f} s, open {m['open_pct_time']:5.1f} % time, "
                  f"saved {m['cooling_drag_saved_Wh_per_100km']:6.1f} Wh/100km, batt {m['battery_Wh_per_100km']:6.0f} Wh/100km "
                  f"({m['battery_saved_pct']:+.1f} %), {m['distance_km']:.1f} km")


if __name__ == "__main__":
    main()
