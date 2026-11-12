"""Digital twin of the SMA brake-duct bench rig.

Runs the real firmware logic (firmware/host/rig_core.dll, built from firmware/sma_rig/rig_core.cpp) against
a physics model of the rig and emulated sensors:

  lamp      500 W R7s halogen, 85 % radiant, filament lag 0.25 s, axis 23 mm from the spring line: 100 % on gives
            143.5 kW/m2 (assumed 110 kW/m2 at 30 mm, line-source 1/r); the car lap peaks at 129 kW/m2
  reflector lumped aluminium, 72 J/K, 0.35 W/K to room, absorbs 15 % of the lamp power (TC3)
  springs   six 0.5 mm NiTiCu wires, emissivity 0.8 (same heat balance as phase2_transient_lap.py):
            rho c_eff D/4 dT/dt = eps G/4 - eps sigma (T^4 - Tamb^4) - h (T - T_local)
            h = max(Churchill-Bernstein forced, Morgan natural); Liang-Rogers kinetics As70/Af76/Ms64/Mf58 C
            with latent heat; stroke from the MAPDL six-spring sweep (fea/pack_NiTiCu)
  air       50 mm blower nozzle: 13 m/s on the pack at 100 % (car: 5.3-11.2 m/s); duct throat 4.7 m/s at 100 %
  load      load cell sees the bias spring: 8.0 N closed, 23.2 N at the MAPDL hot stroke (15.15 mm)
Sensors: MAX31855 0.25 C steps at 10 Hz (TC1 bead lag 0.4 s), HX711 10 Hz with noise, AS5600 12-bit,
fan tach 2 pulses/rev, MPXV7002DP on a 10-bit ADC.

Usage:
  python rig_twin.py serve [--port 7777] [--speed 1.0]      # TCP "serial port" for the Phase 4 logger
  python rig_twin.py script cmds.txt out.jsonl               # accelerated headless run (see campaign)
"""
from __future__ import annotations

import argparse
import ctypes as C
import json
import math
import os
import random
import socket
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DLL = os.path.join(HERE, "..", "host", "rig_core.dll")
SIGMA = 5.670374419e-8


class Inputs(C.Structure):
    _fields_ = [("now_ms", C.c_uint32), ("t_spring_c", C.c_float), ("t_air_c", C.c_float),
                ("t_heater_c", C.c_float), ("t_spring_ok", C.c_uint8), ("t_air_ok", C.c_uint8),
                ("t_heater_ok", C.c_uint8), ("load_ready", C.c_uint8), ("load_raw", C.c_int32),
                ("angle_ok", C.c_uint8), ("angle_raw", C.c_uint16), ("tach_count", C.c_uint32),
                ("pitot_adc", C.c_uint16), ("button", C.c_uint8)]


class Outputs(C.Structure):
    _fields_ = [("heater_on", C.c_uint8), ("fan_pct", C.c_uint8), ("led_on", C.c_uint8)]


class Firmware:
    def __init__(self, now_ms=0):
        self.lib = C.CDLL(DLL)
        self.lib.rig_new.restype = C.c_void_p
        self.lib.rig_new.argtypes = [C.c_uint32]
        for name in ("rig_free",):
            getattr(self.lib, name).argtypes = [C.c_void_p]
        self.lib.rig_command.argtypes = [C.c_void_p, C.c_char_p]
        self.lib.rig_update.argtypes = [C.c_void_p, C.POINTER(Inputs), C.POINTER(Outputs)]
        self.lib.rig_telemetry.argtypes = [C.c_void_p, C.c_char_p, C.c_int]
        self.lib.rig_message.argtypes = [C.c_void_p, C.c_char_p, C.c_int]
        if self.lib.rig_sizeof_inputs() != C.sizeof(Inputs) or self.lib.rig_sizeof_outputs() != C.sizeof(Outputs):
            raise RuntimeError("struct layout mismatch between rig_core.dll and the twin")
        self.h = self.lib.rig_new(now_ms)
        self.buf = C.create_string_buffer(256)

    def command(self, line: str):
        self.lib.rig_command(self.h, line.strip().encode())

    def update(self, inp: Inputs) -> Outputs:
        out = Outputs()
        self.lib.rig_update(self.h, C.byref(inp), C.byref(out))
        return out

    def lines(self):
        out = []
        if self.lib.rig_message(self.h, self.buf, 256):
            out.append(self.buf.value.decode())
        if self.lib.rig_telemetry(self.h, self.buf, 256):
            out.append(self.buf.value.decode())
        return out


# ------------------------------------------------------------------ physics
class Plant:
    T_AMB = 296.15
    D_W, RHO_W, CP_W, DH, EPS = 0.5e-3, 6450.0, 837.0, 20e3, 0.8
    AS, AF, MS, MF = (v + 273.15 for v in (70.0, 76.0, 64.0, 58.0))
    P_EL, RAD_FRAC, K_G = 500.0, 0.85, 143.5e3 / (0.85 * 500.0)   # lamp axis 23 mm from the springs
    F0, F_HOT, S_HOT = 8.0, 23.2, 15.153
    CRANK_R, PHI0 = 10.89, math.radians(-40.0)

    def __init__(self, seed=1, wire_mm=0.5, eps=0.8):
        self.rng = random.Random(seed)
        self.D_W = wire_mm / 1000.0
        self.EPS = eps
        self.p_rad = 0.0
        self.t_refl = self.T_AMB
        self.t_wire = self.T_AMB
        self.xi, self.branch = 1.0, "cool"
        self.h_scale, self.c_scale, self.xi_branch_start = 1.0, 0.0, 1.0
        self.stroke = 0.0
        self.fan_rpm = 0.0
        self.tach = 0.0
        sweep = json.load(open(os.path.join(ROOT, "fea", "pack_NiTiCu", "phase3_results.json")))["sweep"]["heating"]
        pairs = sorted({(round(self._xi_heat(tc + 273.15, 1.0), 4), max(s, 0.0)) for tc, s, ok in sweep if ok})
        self.xi_tab = [p[0] for p in pairs]
        self.s_tab = [p[1] for p in pairs]

    # Liang-Rogers branches (same as phase2_transient_lap.py)
    def _xi_heat(self, T, xi0):
        if T <= self.AS:
            return xi0
        if T >= self.AF:
            return 0.0
        return 0.5 * xi0 * (math.cos(math.pi * (T - self.AS) / (self.AF - self.AS)) + 1)

    def _xi_cool(self, T, xi0):
        if T >= self.MS:
            return xi0
        if T <= self.MF:
            return 1.0
        return 0.5 * (1 - xi0) * math.cos(math.pi * (T - self.MF) / (self.MS - self.MF)) + 0.5 * (1 + xi0)

    # Hysteresis with return-point memory.
    # Heating branch:  xi = h_scale * g_h(T)           g_h = major heating curve (1 below As, 0 above Af)
    # Cooling branch:  xi = 1 - c_scale * (1 - g_c(T))  g_c = major cooling curve (0 above Ms, 1 below Mf)
    # A reversal only re-anchors the new branch through (T, xi) if the branch being left actually
    # transformed material. Temperature ripple inside the dead band (no reverse transformation) keeps the
    # old branch, so the wire returns to the same curve, as real SMA does. Re-anchoring at every ripple
    # (the plain Liang-Rogers reversal rule) ratchets xi down with each 200 ms heater window.
    def _reverse(self, new_branch, T):
        moved = abs(self.xi - self.xi_branch_start) > 1e-6
        if moved:
            if new_branch == "heat":
                g = self._xi_heat(T, 1.0)
                self.h_scale = self.xi / g if g > 1e-9 else 0.0
            else:
                g = self._xi_cool(T, 0.0)
                self.c_scale = (1 - self.xi) / (1 - g) if g < 1 - 1e-9 else 0.0
        self.branch = new_branch
        self.xi_branch_start = self.xi

    def _branch_fn(self):
        if self.branch == "heat":
            return lambda T: self.h_scale * self._xi_heat(T, 1.0)
        return lambda T: 1 - self.c_scale * (1 - self._xi_cool(T, 0.0))

    def _h(self, T, u):
        tf = 0.5 * (T + self.T_AMB)
        k = 0.0241 * (tf / 273.15) ** 0.81
        mu = 1.716e-5 * (tf / 273.15) ** 1.5 * (273.15 + 110.4) / (tf + 110.4)
        nu = mu / (101325 / (287.05 * tf))
        pr = 0.71
        D = self.D_W
        re = max(u, 0.0) * D / nu
        forced = (0.3 + 0.62 * re ** 0.5 * pr ** (1 / 3) / (1 + (0.4 / pr) ** (2 / 3)) ** 0.25 *
                  (1 + (re / 282000) ** 0.625) ** 0.8) * k / D if re > 0 else 0.0
        ra = 9.81 * (1 / tf) * abs(T - self.T_AMB) * D ** 3 / (nu * nu / pr) + 1e-12   # Rayleigh, nu^2/pr = nu*alpha
        natural = 0.85 * ra ** 0.188 * k / D if ra < 100 else 0.48 * ra ** 0.25 * k / D   # Morgan
        return max(forced, natural)

    def stroke_of_xi(self, xi):
        if xi > 0.999:
            return 0.0
        xs, ss = self.xi_tab, self.s_tab
        if xi <= xs[0]:
            return ss[0]
        for k in range(1, len(xs)):
            if xi <= xs[k]:
                f = (xi - xs[k - 1]) / (xs[k] - xs[k - 1])
                return ss[k - 1] + f * (ss[k] - ss[k - 1])
        return ss[-1]

    def step(self, dt, heater_on, fan_pct):
        # lamp filament and reflector
        target = self.RAD_FRAC * self.P_EL if heater_on else 0.0
        self.p_rad += (target - self.p_rad) * min(dt / 0.25, 1.0)
        p_in = self.p_rad / self.RAD_FRAC
        self.t_refl += dt * (0.15 * p_in - 0.35 * (self.t_refl - self.T_AMB)) / 72.0
        # fan
        f = fan_pct / 100.0
        rpm_target = 0.0 if fan_pct < 10 else 600 + 2400 * f
        self.fan_rpm += (rpm_target - self.fan_rpm) * min(dt / 1.0, 1.0)
        self.tach += self.fan_rpm / 60.0 * 2.0 * dt
        spin = self.fan_rpm / 3000.0
        u_pack = 13.0 * spin                      # 50 mm blower nozzle on the spring pack
        self.v_throat = 4.7 * spin
        # spring wire
        G = self.K_G * self.p_rad
        T0 = self.t_wire
        t_local = self.T_AMB + 0.05 * (self.t_refl - self.T_AMB)
        q = (self.EPS * G / 4 - self.EPS * SIGMA * (T0 ** 4 - self.T_AMB ** 4)
             - self._h(T0, u_pack) * (T0 - t_local))
        heating = q > 0
        if heating != (self.branch == "heat"):
            self._reverse("heat" if heating else "cool", T0)
        fx = self._branch_fn()
        dxdT = (fx(T0 + 0.05) - fx(T0 - 0.05)) / 0.1
        cp_eff = self.CP_W + self.DH * abs(dxdT)
        self.t_wire = T0 + dt * q / (self.RHO_W * cp_eff * self.D_W / 4)
        new = fx(self.t_wire)
        self.xi = min(self.xi, new) if self.branch == "heat" else max(self.xi, new)
        s_target = self.stroke_of_xi(self.xi)
        self.stroke += (s_target - self.stroke) * min(dt / 0.05, 1.0)

    def force(self):
        return self.F0 + (self.F_HOT - self.F0) / self.S_HOT * self.stroke

    def angle_deg(self):
        a = self.stroke / self.CRANK_R + math.sin(self.PHI0)
        return math.degrees(math.asin(max(-1.0, min(1.0, a))) - self.PHI0)


class Sensors:
    """Emulates the shield's sensors, with realistic update rates, lag, resolution and noise."""

    def __init__(self, plant: Plant, seed=2):
        self.p = plant
        self.rng = random.Random(seed)
        self.tc1_bead = plant.t_wire
        self.tc3_lag = plant.t_refl
        self.last_tc = -1.0
        self.last_hx = -1.0
        self.tc_vals = (23.0, 23.0, 23.0)
        self.hx_ready = False
        self.hx_raw = 0
        self.HX_OFFSET, self.HX_CAL = 81234, 42800.0
        self.AS_ZERO = 1234
        self.tc1_open = False

    def read(self, t, dt) -> Inputs:
        p, r = self.p, self.rng
        self.tc1_bead += (p.t_wire - self.tc1_bead) * min(dt / 0.4, 1.0)
        self.tc3_lag += (p.t_refl - self.tc3_lag) * min(dt / 2.0, 1.0)
        inp = Inputs()
        inp.now_ms = int(t * 1000)
        if t - self.last_tc >= 0.1:
            self.last_tc = t

            def q(v):
                return round((v - 273.15 + r.gauss(0, 0.1)) * 4) / 4
            self.tc_vals = (q(self.tc1_bead), q(p.T_AMB + 0.002 * (p.t_refl - p.T_AMB)), q(self.tc3_lag))
        inp.t_spring_c, inp.t_air_c, inp.t_heater_c = self.tc_vals
        inp.t_spring_ok = inp.t_air_ok = inp.t_heater_ok = 1
        if self.tc1_open:                      # MAX31855 reports an open thermocouple as NaN
            inp.t_spring_c, inp.t_spring_ok = float("nan"), 0
        inp.load_ready = 0
        if t - self.last_hx >= 0.1:
            self.last_hx = t
            inp.load_ready = 1
            inp.load_raw = int(self.HX_OFFSET + p.force() * self.HX_CAL + r.gauss(0, 400))
        inp.angle_ok = 1
        inp.angle_raw = int(round(self.AS_ZERO + p.angle_deg() * 4096 / 360 + r.choice((-1, 0, 0, 1)))) % 4096
        inp.tach_count = int(p.tach)
        dp = 0.5 * 1.2 * p.v_throat ** 2 if hasattr(p, "v_throat") else 0.0
        inp.pitot_adc = max(0, min(1023, int(round(511 + dp / 5000 * 1023 + r.choice((-1, 0, 1))))))
        inp.button = 0
        return inp


class Rig:
    DT = 0.01

    def __init__(self, seed=1, wire_mm=0.5, eps=0.8):
        self.plant = Plant(seed, wire_mm, eps)
        self.sensors = Sensors(self.plant, seed + 1)
        self.fw = Firmware(0)
        self.t = 0.0
        self.out = Outputs()

    def step(self):
        inp = self.sensors.read(self.t, self.DT)
        self.out = self.fw.update(inp)
        self.plant.step(self.DT, self.out.heater_on, self.out.fan_pct)
        self.t += self.DT
        return self.fw.lines()

    def truth(self):
        p = self.plant
        return {"t": round(self.t, 3), "T_wire_C": round(p.t_wire - 273.15, 3), "xi": round(p.xi, 4),
                "stroke_mm": round(p.stroke, 4), "angle_deg": round(p.angle_deg(), 3),
                "force_N": round(p.force(), 3), "T_refl_C": round(p.t_refl - 273.15, 2)}


# ------------------------------------------------------------------ modes
def serve(port, speed):
    """Real-time twin behind a TCP socket: the Phase 4 logger connects with --port socket://localhost:PORT."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"twin listening on socket://localhost:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        conn.setblocking(False)
        rig = Rig()
        rx = b""
        t_wall0 = time.perf_counter()
        print("client connected", flush=True)
        try:
            while True:
                target = (time.perf_counter() - t_wall0) * speed
                while rig.t < target:
                    for line in rig.step():
                        conn.sendall((line + "\n").encode())
                try:
                    data = conn.recv(4096)
                    if data == b"":
                        break
                    rx += data
                    while b"\n" in rx:
                        cmd, rx = rx.split(b"\n", 1)
                        if cmd.strip():
                            rig.fw.command(cmd.decode(errors="replace"))
                            print("cmd", cmd.decode(errors="replace").strip(), "at", round(rig.t, 2), flush=True)
                except BlockingIOError:
                    pass
                time.sleep(0.002)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass
        print("client disconnected", flush=True)
        conn.close()


def run_script(script, seed=1, wire_mm=0.5, eps=0.8, truth_every=0.05, messages=None):
    """script: list of (t_s, command or None). Commands starting with '!' are fault injections for the twin
    (!TC1_OPEN, !TC1_OK). Runs until the last time. Returns (telemetry dicts, truth dicts)."""
    rig = Rig(seed, wire_mm, eps)
    tel, tru = [], []
    events = sorted(script, key=lambda e: e[0])
    k = 0
    t_end = events[-1][0]
    next_truth = 0.0
    while rig.t < t_end:
        while k < len(events) and events[k][0] <= rig.t:
            cmd = events[k][1]
            if cmd and cmd.startswith("!"):
                rig.sensors.tc1_open = cmd == "!TC1_OPEN"
            elif cmd:
                rig.fw.command(cmd)
            k += 1
        for line in rig.step():
            if line.startswith("{"):
                tel.append(json.loads(line))
            elif messages is not None:
                messages.append((round(rig.t, 2), line))
        if rig.t >= next_truth:
            tru.append(rig.truth())
            next_truth += truth_every
    return tel, tru


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--port", type=int, default=7777)
    s.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args()
    if a.mode == "serve":
        serve(a.port, a.speed)
