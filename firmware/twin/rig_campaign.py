"""Virtual test campaign on the rig digital twin (firmware DLL + physics + emulated sensors).

Every number below comes from the firmware's own telemetry (what the real logger would record), and is
compared with the twin's internal "truth" so sensor and firmware errors are visible.

  T1 hysteresis   slow PI ramp 45 -> 90 -> 45 C at 2 C/min: stroke vs spring temperature
  T2 step         HEAT 100 then COOL, 0.5 mm vs 1.2 mm wire: time to 90 % open / to 10 % closed
  T3 lap          LAP 3 (5 braking zones per lap): open/close per zone, time open on straights
  T4 endurance    1000 heat/cool cycles (8 s / 12 s): firmware stability and cycle-to-cycle stroke
  T5 faults       open thermocouple mid-heat; heater left at 100 % (spring over-temperature cut-out)
Writes bench/virtual/*.json, *.png, and a JSONL telemetry log per test.
"""
from __future__ import annotations

import json
import math
import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from rig_twin import ROOT, run_script  # noqa: E402

OUT = os.path.join(ROOT, "bench", "virtual")
AS, AF, MS, MF, S_HOT = 70.0, 76.0, 64.0, 58.0, 15.153
LABEL = "DIGITAL TWIN - not measured data"


def arr(rows, key):
    return np.array([np.nan if r.get(key) is None else r[key] for r in rows], dtype=float)


def save_jsonl(name, rows):
    with open(os.path.join(OUT, name), "w") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")


def crossing(x, y, level, rising):
    for i in range(1, len(y)):
        a, b = y[i - 1], y[i]
        if (rising and a < level <= b) or (not rising and a > level >= b):
            f = (level - a) / (b - a) if b != a else 0
            return float(x[i - 1] + f * (x[i] - x[i - 1]))
    return float("nan")


def watermark(fig):
    fig.text(0.5, 0.5, "DIGITAL TWIN", fontsize=54, color="tab:purple", alpha=0.10, ha="center", va="center",
             rotation=20)


# ---------------------------------------------------------------- T1
def t1_hysteresis():
    script = [(0, None), (2, "FAN 30"), (2.5, "HOLD 45"), (90, "RAMP 90 2"), (90 + 22.5 * 60 + 60, "RAMP 45 2"),
              (90 + 2 * 22.5 * 60 + 180, None)]
    tel, tru = run_script(script)
    save_jsonl("T1_hysteresis.jsonl", tel)
    t = arr(tel, "timestamp_ms") / 1000
    T = arr(tel, "temp_C")
    s = arr(tel, "stroke_mm")
    t_up = 90 + 22.5 * 60 + 60
    up = (t > 90) & (t < t_up)
    dn = t >= t_up
    res = {
        "heating_5pct_C": crossing(T[up], s[up], 0.05 * S_HOT, True),
        "heating_50pct_C": crossing(T[up], s[up], 0.5 * S_HOT, True),
        "heating_95pct_C": crossing(T[up], s[up], 0.95 * S_HOT, True),
        "cooling_95pct_C": crossing(T[dn], s[dn], 0.95 * S_HOT, False),
        "cooling_50pct_C": crossing(T[dn], s[dn], 0.5 * S_HOT, False),
        "cooling_5pct_C": crossing(T[dn], s[dn], 0.05 * S_HOT, False),
        "max_stroke_mm": float(np.nanmax(s)), "max_force_N": float(np.nanmax(arr(tel, "force_N"))),
        "max_angle_deg": float(np.nanmax(arr(tel, "angle_deg"))),
    }
    res["hysteresis_width_50pct_C"] = res["heating_50pct_C"] - res["cooling_50pct_C"]
    tt = np.array([r["t"] for r in tru])
    Tw = np.array([r["T_wire_C"] for r in tru])
    Tm = np.interp(tt, t, T)
    res["tc1_minus_wire_rms_C"] = float(np.sqrt(np.nanmean((Tm[tt > 120] - Tw[tt > 120]) ** 2)))
    sp = arr(tel, "setpoint_C")
    m = ~np.isnan(sp) & (t > 120)
    res["pi_tracking_rms_C"] = float(np.sqrt(np.nanmean((T[m] - sp[m]) ** 2)))
    res["pi_tracking_max_C"] = float(np.nanmax(np.abs(T[m] - sp[m])))

    fig, ax = plt.subplots(1, 2, figsize=(14, 5.5))
    ax[0].plot(T[up], s[up], color="tab:red", lw=1.6, label="Heating (twin telemetry)")
    ax[0].plot(T[dn], s[dn], color="tab:blue", lw=1.6, label="Cooling (twin telemetry)")
    for v, c, n in ((AS, "tab:red", "As"), (AF, "tab:red", "Af"), (MS, "tab:blue", "Ms"), (MF, "tab:blue", "Mf")):
        ax[0].axvline(v, color=c, ls=":", lw=1)
        ax[0].text(v, S_HOT * 1.08, n, color=c, ha="center", fontsize=9)
    ax[0].set_xlabel("Spring temperature, TC1 [°C]")
    ax[0].set_ylabel("Rod stroke from AS5600 angle [mm]")
    ax[0].set_title(f"T1 hysteresis, 2 °C/min ramp: width {res['hysteresis_width_50pct_C']:.1f} °C at 50 % stroke")
    ax[0].set_ylim(-1, S_HOT * 1.2)
    ax[0].grid(alpha=0.3)
    ax[0].legend(loc="center right")
    ax[1].plot(t / 60, T, color="tab:red", lw=1.2, label="TC1 spring")
    ax[1].plot(t / 60, sp, color="k", ls="--", lw=1, label="Setpoint (PI)")
    ax2 = ax[1].twinx()
    ax2.plot(t / 60, arr(tel, "heater_pct"), color="tab:orange", lw=0.6, alpha=0.6, label="Heater duty")
    ax2.set_ylabel("Heater duty [%]")
    ax[1].set_xlabel("Time [min]")
    ax[1].set_ylabel("Temperature [°C]")
    ax[1].set_title(f"Firmware PI tracking: RMS {res['pi_tracking_rms_C']:.2f} °C")
    ax[1].grid(alpha=0.3)
    ax[1].legend(loc="upper left")
    watermark(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "T1_hysteresis.png"), dpi=150)
    plt.close(fig)
    return res


# ---------------------------------------------------------------- T2
def t2_step():
    res = {}
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.5))
    for d, col in ((0.5, "tab:green"), (1.2, "tab:gray")):
        script = [(0, None), (2, "FAN 30"), (10, "HEAT 100"), (19, "COOL"), (60, None)]
        tel, tru = run_script(script, wire_mm=d)
        save_jsonl(f"T2_step_d{d}.jsonl", tel)
        t = arr(tel, "timestamp_ms") / 1000
        s = arr(tel, "stroke_mm")
        T = arr(tel, "temp_C")
        h = (t >= 10) & (t < 19)
        c = t >= 19
        smax = float(np.nanmax(s))
        r = {"max_stroke_mm": smax, "peak_temp_C": float(np.nanmax(T)),
             "open_90pct_s": crossing(t[h], s[h], 0.9 * S_HOT, True) - 10,
             "close_10pct_s": crossing(t[c], s[c], 0.1 * S_HOT, False) - 19,
             "fault": int(max(x["fault"] for x in tel))}
        res[f"wire_{d}mm"] = r
        ax[0].plot(t, s, color=col, lw=1.6, label=f"{d} mm wire")
        ax[1].plot(t, T, color=col, lw=1.6, label=f"{d} mm wire")
    for a, yl in ((ax[0], "Stroke [mm]"), (ax[1], "TC1 spring temperature [°C]")):
        a.axvspan(10, 19, color="tab:red", alpha=0.06)
        a.set_xlabel("Time [s] (heater 100 % from 10 s to 19 s)")
        a.set_ylabel(yl)
        a.grid(alpha=0.3)
        a.legend()
    a5, a12 = res["wire_0.5mm"], res["wire_1.2mm"]
    ax[0].set_title(f"T2 step: open in {a5['open_90pct_s']:.1f} s (0.5 mm) vs {a12['open_90pct_s']:.1f} s (1.2 mm)")
    ax[1].set_title("Spring temperature (over-temperature cut-out at 110 °C)")
    watermark(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "T2_step_response.png"), dpi=150)
    plt.close(fig)
    res["speedup_open"] = a12["open_90pct_s"] / a5["open_90pct_s"]
    res["speedup_close"] = a12["close_10pct_s"] / a5["close_10pct_s"]
    return res


# ---------------------------------------------------------------- T3
def t3_lap():
    env = json.load(open(os.path.join(ROOT, "cfd", "transient", "spring_env_NiTiCu_Af76_gap3_eps0.8_d0.5.json")))
    lap_s = len(env["t_s"]) * env["dt_s"]
    script = [(0, None), (2, "LAP 3"), (2 + 3 * lap_s + 20, None)]
    tel, tru = run_script(script)
    save_jsonl("T3_lap.jsonl", tel)
    t = arr(tel, "timestamp_ms") / 1000
    ang = arr(tel, "angle_deg")
    fan = arr(tel, "fan_pct")
    heat = arr(tel, "heater_pct")
    last = (t >= 2 + 2 * lap_s) & (t < 2 + 3 * lap_s)
    straight = last & (fan >= np.nanmax(fan[last]) - 1)      # blower duty follows car speed: top = straights
    zone = last & ~straight
    is_open = ang >= 40
    # the car lap model with the same spring inputs: compare louver angle over the last lap
    t_car = np.array(env["t_s"])
    ang_car = np.array(env["angle_deg"])
    ang_rig = np.interp(t_car, t[last] - (2 + 2 * lap_s), ang[last])
    car_open = ang_car >= 40
    events, prev = [], False
    for ti, o in zip(t[last], is_open[last]):
        if o != prev:
            events.append({"t_in_lap_s": round(float(ti - (2 + 2 * lap_s)), 2), "event": "open" if o else "close"})
            prev = o
    res = {"lap_s": lap_s, "open_pct_straights": float(np.mean(is_open[straight]) * 100),
           "open_pct_braking_and_corners": float(np.mean(is_open[zone]) * 100),
           "openings_per_lap": sum(e["event"] == "open" for e in events),
           "temp_range_C": [float(np.nanmin(arr(tel, "temp_C")[last])), float(np.nanmax(arr(tel, "temp_C")[last]))],
           "events_last_lap": events, "fault": int(max(x["fault"] for x in tel)),
           "car_model_openings_per_lap": int(np.sum(np.diff(car_open.astype(int)) == 1) + (1 if car_open[0] else 0)),
           "car_model_open_pct_of_lap": float(np.mean(car_open) * 100),
           "rig_open_pct_of_lap": float(np.mean(ang_rig >= 40) * 100),
           "rig_vs_car_open_state_agreement_pct": float(np.mean((ang_rig >= 40) == car_open) * 100),
           "rig_vs_car_angle_rms_deg": float(np.sqrt(np.mean((ang_rig - ang_car) ** 2)))}
    fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    ax[0].plot(t, heat, color="tab:orange", lw=0.8, label="Lamp duty (rotor radiation on the spring)")
    ax[0].plot(t, fan, color="tab:blue", lw=0.8, label="Blower duty (airflow at the spring)")
    ax[0].set_ylabel("%")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, arr(tel, "temp_C"), color="tab:red", lw=1.2)
    for v, c in ((AF, "tab:red"), (MF, "tab:blue")):
        ax[1].axhline(v, color=c, ls=":", lw=1)
    ax[1].set_ylabel("TC1 spring [°C]")
    ax[2].plot(t, ang, color="tab:green", lw=1.2, label="Rig twin (firmware telemetry)")
    for k in range(3):
        ax[2].plot(t_car + 2 + k * lap_s, ang_car, color="k", ls="--", lw=0.9,
                   label="Car lap model, same spring inputs" if k == 0 else None)
    ax[2].legend(loc="upper right")
    ax[2].set_ylabel("Louver angle [°]")
    ax[2].set_xlabel("Time [s]")
    ax[0].set_title(f"T3 car lap on the rig x3: {res['openings_per_lap']} openings per lap (car model "
                    f"{res['car_model_openings_per_lap']}), open/closed state agrees "
                    f"{res['rig_vs_car_open_state_agreement_pct']:.0f} % of the last lap")
    for a in ax:
        a.grid(alpha=0.3)
    watermark(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "T3_lap_profile.png"), dpi=150)
    plt.close(fig)
    return res


# ---------------------------------------------------------------- T4
def t4_endurance(n=1000, period=25.0, heat_s=10.0, hold_c=90):
    script = [(0, None), (2, "FAN 20")]
    t0 = 5.0
    for k in range(n):
        script += [(t0 + period * k, f"HOLD {hold_c}"), (t0 + period * k + heat_s, "COOL"),
                   (t0 + period * k + period - 0.5, "FAN 20")]
    script.append((t0 + period * n + 5, None))
    wall = time.time()
    tel, _ = run_script(script, truth_every=1e9)
    wall = time.time() - wall
    t = arr(tel, "timestamp_ms") / 1000
    s = arr(tel, "stroke_mm")
    peaks, troughs = [], []
    for k in range(n):
        m = (t >= t0 + period * k) & (t < t0 + period * (k + 1))
        peaks.append(float(np.nanmax(s[m])))
        troughs.append(float(np.nanmin(s[m])))
    gaps = np.diff(arr(tel, "timestamp_ms"))
    res = {"cycles": n, "cycle": f"HOLD {hold_c} C for {heat_s:g} s, COOL, blower 20 % between", "rig_time_h": (t0 + period * n) / 3600,
           "twin_wall_time_s": wall,
           "telemetry_lines": len(tel), "telemetry_gap_max_ms": float(gaps.max()), "telemetry_gap_min_ms": float(gaps.min()),
           "faults": int(max(x["fault"] for x in tel)),
           "peak_stroke_min_mm": min(peaks), "peak_stroke_max_mm": max(peaks),
           "closed_stroke_max_mm": max(troughs),
           "cycles_fully_open": int(sum(p >= 0.95 * S_HOT for p in peaks)),
           "cycles_fully_closed": int(sum(tr <= 0.05 * S_HOT for tr in troughs))}
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(range(1, n + 1), peaks, ".", ms=2, color="tab:red", label="Open stroke per cycle")
    ax.plot(range(1, n + 1), troughs, ".", ms=2, color="tab:blue", label="Closed stroke per cycle")
    ax.set_xlabel("Cycle")
    ax.set_ylabel("Stroke [mm]")
    ax.set_title(f"T4 endurance: {n} cycles ({res['rig_time_h']:.1f} h of rig time) - the twin has no fatigue model; "
                 "this checks firmware and procedure only")
    ax.grid(alpha=0.3)
    ax.legend()
    watermark(fig)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "T4_endurance.png"), dpi=150)
    plt.close(fig)
    return res


# ---------------------------------------------------------------- T5
def t5_faults():
    res = {}
    msgs = []
    tel, _ = run_script([(0, None), (2, "FAN 30"), (5, "HEAT 100"), (8, "!TC1_OPEN"), (12, "!TC1_OK"),
                         (14, "HEAT 50"), (16, "RESET"), (18, "HEAT 30"), (25, None)], messages=msgs)
    t = arr(tel, "timestamp_ms") / 1000
    h = arr(tel, "heater_pct")
    f = np.array([r["fault"] for r in tel])
    res["tc_open"] = {"fault_raised_at_s": float(t[np.argmax(f > 0)]),
                      "heater_after_fault_pct": float(np.nanmax(h[(t > 8.1) & (t < 16)])),
                      "heat_refused_while_latched": any("refused" in m for _, m in msgs),
                      "heater_after_reset_pct": float(np.nanmax(h[t > 18.2])),
                      "messages": msgs}
    msgs = []
    # blower off (blocked or failed) with the lamp at 100 %: the spring must hit the 110 C cut-out
    tel, _ = run_script([(0, None), (2, "FAN 0"), (5, "HEAT 100"), (40, None)], messages=msgs)
    t = arr(tel, "timestamp_ms") / 1000
    T = arr(tel, "temp_C")
    f = np.array([r["fault"] for r in tel])
    if not (f > 0).any():
        raise RuntimeError("over-temperature case did not trip")
    k = int(np.argmax(f > 0))
    res["over_temperature"] = {"scenario": "blower off, lamp 100 %", "trip_at_s": float(t[k]), "tc1_at_trip_C": float(T[k]),
                               "peak_tc1_C": float(np.nanmax(T)),
                               "heater_after_trip_pct": float(np.nanmax(arr(tel, "heater_pct")[k + 1:])),
                               "fan_after_trip_pct": float(np.nanmin(arr(tel, "fan_pct")[k + 1:])), "messages": msgs}
    return res


def main():
    os.makedirs(OUT, exist_ok=True)
    out = {"data_source": LABEL, "firmware": "firmware/sma_rig/rig_core.cpp (host build rig_core.dll)"}
    for name, fn in (("T1_hysteresis", t1_hysteresis), ("T2_step", t2_step), ("T3_lap", t3_lap),
                     ("T5_faults", t5_faults), ("T4_endurance", t4_endurance)):
        w = time.time()
        out[name] = fn()
        print(name, f"{time.time() - w:.0f} s", json.dumps(out[name], default=float)[:600], flush=True)
    json.dump(out, open(os.path.join(OUT, "virtual_campaign_results.json"), "w"), indent=2, default=float)
    print("WROTE", os.path.join(OUT, "virtual_campaign_results.json"))


if __name__ == "__main__":
    main()

