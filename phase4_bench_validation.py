#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
phase4_bench_validation.py
==========================
F1 Thermo-Kinetic Adaptive Brake Duct - Phase 4 physical bench validation.

Connects to the ESP32/Arduino DAQ over USB serial and logs temperature, force
and linear stroke at 20 Hz for a 120 s thermal cycle (60 s heating, then 60 s
cooling). Every sample is streamed to CSV and drawn on a live 3-panel
dashboard, and the bench data is correlated against the Phase 3 ANSYS
prediction.

Telemetry contract (one JSON object per line, 115200 baud):
    {"temp_C": 84.5, "force_N": 31.2, "stroke_mm": 14.2, "timestamp_ms": 12450}

FEA reference
-------------
A PNG plot cannot be turned back into numbers, so the Phase 3 prediction is
rebuilt as a lookup model with the Phase 3 Liang-Rogers transformation kinetics:
As = 75 C, Af = 85 C, Ms = 55 C, Mf = 45 C, stroke_max = 14.5 mm,
force_max = 32.0 N. Pass --fea-csv to use a tabulated FEA export instead
(columns: temp_C, stroke_mm, force_N, branch = heating | cooling).

Usage
-----
    python phase4_bench_validation.py                  # auto-detect DAQ port
    python phase4_bench_validation.py --port COM5
    python phase4_bench_validation.py --no-live        # headless, no GUI window
    python phase4_bench_validation.py --simulate       # dry run without hardware
                                                       # (all data tagged SIMULATED)
Outputs
-------
    bench/Phase4_Bench_Test_Data.csv
    bench/Phase4_Bench_Dashboard.png
    bench/Phase4_Correlation_Summary.json   (consumed by Phase 5)

Requirements: pyserial, numpy, matplotlib.
"""

from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import argparse
import csv
import json
import logging
import math
import os
import random
import sys
import threading
import time
from datetime import datetime

import matplotlib
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

LOG = logging.getLogger("phase4_bench")

# =============================================================================
# CONFIGURATION
# =============================================================================
CSV_PATH = PROJECT_ROOT + r"\bench\Phase4_Bench_Test_Data.csv"
DASHBOARD_PATH = PROJECT_ROOT + r"\bench\Phase4_Bench_Dashboard.png"
SUMMARY_PATH = PROJECT_ROOT + r"\bench\Phase4_Correlation_Summary.json"
BAUD_RATE = 115200
SAMPLE_HZ = 20.0
HEATING_S = 60.0
COOLING_S = 60.0
STALE_LIMIT_S = 0.5
DASHBOARD_EVERY = 4
CSV_FLUSH_EVERY = 20

FEA_AS, FEA_AF, FEA_MS, FEA_MF = 75.0, 85.0, 55.0, 45.0
FEA_STROKE_MAX_MM = 14.5
FEA_FORCE_MAX_N = 32.0

# Phase 3 external crank (louver angle from rod stroke)
CRANK_PHASE0 = -math.radians(40.0)
CRANK_R_MM = 14.0 / (2.0 * math.sin(math.radians(40.0)))

ONSET_FRACTION = 0.05
ONSET_MIN_MM = 0.5
ONSET_HOLD_SAMPLES = 5
SMOOTH_WINDOW = 5

REQUIRED_KEYS = ("temp_C", "force_N", "stroke_mm", "timestamp_ms")
KNOWN_USB_BRIDGES = (
    (0x10C4, 0xEA60, "Silicon Labs CP210x"),
    (0x1A86, 0x7523, "WCH CH340"),
    (0x1A86, 0x55D4, "WCH CH9102"),
    (0x0403, 0x6001, "FTDI FT232R"),
    (0x0403, 0x6015, "FTDI FT231X"),
    (0x303A, None, "Espressif native USB"),
    (0x2341, None, "Arduino"),
    (0x2A03, None, "Arduino.org"),
)


# =============================================================================
# TELEMETRY PARSING
# =============================================================================
class Packet:
    __slots__ = ("temp_c", "force_n", "stroke_mm", "timestamp_ms")

    def __init__(self, temp_c: float, force_n: float, stroke_mm: float, timestamp_ms: int) -> None:
        self.temp_c = temp_c
        self.force_n = force_n
        self.stroke_mm = stroke_mm
        self.timestamp_ms = timestamp_ms


def parse_packet(text: str) -> Packet | None:
    try:
        obj = json.loads(text)
    except (json.JSONDecodeError, ValueError):
        return None
    if not isinstance(obj, dict) or any(key not in obj for key in REQUIRED_KEYS):
        return None
    try:
        values = [float(obj[key]) for key in REQUIRED_KEYS]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return Packet(values[0], values[1], values[2], int(values[3]))


# =============================================================================
# TELEMETRY SOURCES
# =============================================================================
class SerialTelemetry:
    source_label = "MEASURED"

    def __init__(self, port: str, baud: int, label: str = "MEASURED") -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial).")
        self.port = port
        self.source_label = label
        if "://" in port:          # e.g. socket://localhost:7777 for the rig digital twin
            self._ser = serial.serial_for_url(port, do_not_open=True)
        else:
            self._ser = serial.Serial()
            self._ser.port = port
        self._ser.baudrate = baud
        self._ser.timeout = 0.1
        self._ser.dtr = False      # avoid ESP32 auto-reset on open
        self._ser.rts = False
        self._lock = threading.Lock()
        self._latest: Packet | None = None
        self._latest_time = 0.0
        self.packets = 0
        self.rejected = 0
        self.timestamp_regressions = 0
        self.error: str | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="serial-reader", daemon=True)

    def start(self) -> None:
        self._ser.open()
        self._ser.reset_input_buffer()
        self._thread.start()
        LOG.info("Serial: opened %s @ %d baud", self.port, self._ser.baudrate)

    def _run(self) -> None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except (serial.SerialException, OSError) as exc:
                self.error = f"Serial read failed on {self.port}: {exc}"
                return
            if not chunk:
                continue
            buffer.extend(chunk)
            while True:
                idx = buffer.find(b"\n")
                if idx < 0:
                    break
                line = bytes(buffer[:idx])
                del buffer[:idx + 1]
                self._handle(line)
            if len(buffer) > 8192:
                buffer.clear()
                self.rejected += 1

    def _handle(self, raw: bytes) -> None:
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            return
        packet = parse_packet(text) if text.startswith("{") else None
        if packet is None:
            self.rejected += 1
            return
        with self._lock:
            if self._latest is not None and packet.timestamp_ms < self._latest.timestamp_ms:
                self.timestamp_regressions += 1
            self._latest = packet
            self._latest_time = time.perf_counter()
            self.packets += 1

    def snapshot(self):
        with self._lock:
            if self._latest is None:
                return None, math.inf
            return self._latest, time.perf_counter() - self._latest_time

    def wait_first(self, timeout: float) -> bool:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if self.packets > 0:
                return True
            if self.error:
                return False
            time.sleep(0.05)
        return self.packets > 0

    def send(self, text: str) -> None:
        self._ser.write((text.strip() + "\n").encode("utf-8"))
        self._ser.flush()
        LOG.info("Serial: sent command %r", text.strip())

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._ser.is_open:
            self._ser.close()


class SimulatedTelemetry:
    """Hardware-free dry run. Data is tagged SIMULATED throughout."""

    source_label = "SIMULATED"

    def __init__(self, heating_s: float, seed: int = 7) -> None:
        self._rng = random.Random(seed)
        self._heating_s = heating_s
        self._bench = LiangRogersFEA(a_s=FEA_AS + 1.8, a_f=FEA_AF + 1.2, m_s=FEA_MS - 1.0, m_f=FEA_MF - 1.5,
                                     stroke_max=14.2, force_max=31.0)
        self._lock = threading.Lock()
        self._latest: Packet | None = None
        self._latest_time = 0.0
        self.packets = 0
        self.rejected = 0
        self.timestamp_regressions = 0
        self.error: str | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-telemetry", daemon=True)
        self._t_peak = 25.0

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._thread.start()
        LOG.warning("SIMULATED telemetry source active - results are NOT physical validation.")

    def _temperature(self, t: float) -> float:
        if t < self._heating_s:
            temp = 98.0 - 73.0 * math.exp(-t / 16.0)
            self._t_peak = temp
            return temp
        return 25.0 + (self._t_peak - 25.0) * math.exp(-(t - self._heating_s) / 14.0)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            t = time.perf_counter() - self._t0
            branch = "heating" if t < self._heating_s else "cooling"
            temp = self._temperature(t)
            xi = self._bench.xi(temp, branch)
            packet = Packet(temp + self._rng.gauss(0.0, 0.15),
                            self._bench.force_max * (1.0 - xi) + self._rng.gauss(0.0, 0.25),
                            self._bench.stroke_max * (1.0 - xi) + self._rng.gauss(0.0, 0.05),
                            int(t * 1000.0))
            with self._lock:
                self._latest = packet
                self._latest_time = time.perf_counter()
                self.packets += 1
            time.sleep(0.02)

    def snapshot(self):
        with self._lock:
            if self._latest is None:
                return None, math.inf
            return self._latest, time.perf_counter() - self._latest_time

    def wait_first(self, timeout: float) -> bool:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline and self.packets == 0:
            time.sleep(0.02)
        return self.packets > 0

    def send(self, text: str) -> None:
        LOG.info("Simulated: command %r acknowledged", text.strip())

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def _usb_label(port) -> str | None:
    for vid, pid, label in KNOWN_USB_BRIDGES:
        if port.vid == vid and (pid is None or port.pid == pid):
            return label
    return None


def probe_port(device: str, baud: int, timeout: float = 3.0) -> bool:
    try:
        ser = serial.Serial()
        ser.port = device
        ser.baudrate = baud
        ser.timeout = 0.1
        ser.dtr = False
        ser.rts = False
        ser.open()
    except (serial.SerialException, OSError):
        return False
    try:
        deadline = time.perf_counter() + timeout
        buffer = bytearray()
        while time.perf_counter() < deadline:
            buffer.extend(ser.read(max(1, ser.in_waiting)))
            while b"\n" in buffer:
                idx = buffer.find(b"\n")
                line = bytes(buffer[:idx]).decode("utf-8", errors="replace").strip()
                del buffer[:idx + 1]
                if line.startswith("{") and parse_packet(line) is not None:
                    return True
        return False
    except (serial.SerialException, OSError):
        return False
    finally:
        ser.close()


def autodetect_port(baud: int) -> str:
    if list_ports is None:
        raise RuntimeError("pyserial is not installed (pip install pyserial).")
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Connect the DAQ board or pass --port.")
    ranked = sorted(ports, key=lambda p: (0 if _usb_label(p) else 1, p.device))
    for port in ranked:
        LOG.info("Probing %s (%s)", port.device, _usb_label(port) or port.description)
        if probe_port(port.device, baud):
            LOG.info("Telemetry detected on %s", port.device)
            return port.device
    raise RuntimeError("No port is streaming valid telemetry JSON: " + ", ".join(p.device for p in ports))


# =============================================================================
# FEA PREDICTION MODELS
# =============================================================================
class LiangRogersFEA:
    label = "Phase 3 FEA lookup (Liang-Rogers kinetics)"

    def __init__(self, a_s=FEA_AS, a_f=FEA_AF, m_s=FEA_MS, m_f=FEA_MF,
                 stroke_max=FEA_STROKE_MAX_MM, force_max=FEA_FORCE_MAX_N) -> None:
        self.a_s, self.a_f, self.m_s, self.m_f = a_s, a_f, m_s, m_f
        self.stroke_max = stroke_max
        self.force_max = force_max

    def xi(self, temp: float, branch: str) -> float:
        if branch == "heating":
            if temp <= self.a_s:
                return 1.0
            if temp >= self.a_f:
                return 0.0
            return 0.5 * (math.cos(math.pi * (temp - self.a_s) / (self.a_f - self.a_s)) + 1.0)
        if temp >= self.m_s:
            return 0.0
        if temp <= self.m_f:
            return 1.0
        return 0.5 * math.cos(math.pi * (temp - self.m_f) / (self.m_s - self.m_f)) + 0.5

    def _eval(self, temps, branches, scale: float) -> np.ndarray:
        temps = np.atleast_1d(np.asarray(temps, dtype=float))
        if isinstance(branches, str):
            branches = [branches] * len(temps)
        return np.array([scale * (1.0 - self.xi(t, b)) for t, b in zip(temps, branches)])

    def stroke(self, temps, branches) -> np.ndarray:
        return self._eval(temps, branches, self.stroke_max)

    def force(self, temps, branches) -> np.ndarray:
        return self._eval(temps, branches, self.force_max)


class TabulatedFEA:
    def __init__(self, path: str) -> None:
        self.label = f"Tabulated FEA ({os.path.basename(path)})"
        tables = {"heating": [], "cooling": []}
        with open(path, newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            missing = {"temp_C", "stroke_mm", "force_N", "branch"} - set(reader.fieldnames or [])
            if missing:
                raise ValueError(f"FEA CSV missing columns: {sorted(missing)}")
            for row in reader:
                branch = row["branch"].strip().lower()
                if branch in tables:
                    tables[branch].append((float(row["temp_C"]), float(row["stroke_mm"]), float(row["force_N"])))
        self._tables = {}
        for branch, rows in tables.items():
            if len(rows) < 2:
                raise ValueError(f"FEA CSV needs at least 2 '{branch}' rows.")
            rows.sort(key=lambda r: r[0])
            arr = np.array(rows)
            self._tables[branch] = (arr[:, 0], arr[:, 1], arr[:, 2])
        self.stroke_max = float(max(t[1].max() for t in self._tables.values()))
        self.force_max = float(max(t[2].max() for t in self._tables.values()))

    def _eval(self, temps, branches, column: int) -> np.ndarray:
        temps = np.atleast_1d(np.asarray(temps, dtype=float))
        if isinstance(branches, str):
            branches = [branches] * len(temps)
        out = np.empty(len(temps))
        for i, (t, b) in enumerate(zip(temps, branches)):
            table = self._tables["heating" if b == "heating" else "cooling"]
            out[i] = np.interp(t, table[0], table[column])
        return out

    def stroke(self, temps, branches) -> np.ndarray:
        return self._eval(temps, branches, 1)

    def force(self, temps, branches) -> np.ndarray:
        return self._eval(temps, branches, 2)


# =============================================================================
# RECORDING
# =============================================================================
class Recorder:
    FIELDS = ("t", "temp", "force", "stroke", "fea_stroke", "fea_force", "age")

    def __init__(self) -> None:
        self.data = {key: [] for key in self.FIELDS}
        self.phase: list = []
        self.stale: list = []
        self.missed_deadlines = 0

    def add(self, t, temp, force, stroke, fea_stroke, fea_force, age, phase, stale) -> None:
        for key, value in zip(self.FIELDS, (t, temp, force, stroke, fea_stroke, fea_force, age)):
            self.data[key].append(value)
        self.phase.append(phase)
        self.stale.append(stale)

    def __len__(self) -> int:
        return len(self.phase)

    def arr(self, key: str) -> np.ndarray:
        return np.asarray(self.data[key], dtype=float)

    def mask(self, phase: str | None = None) -> np.ndarray:
        m = ~np.asarray(self.stale, dtype=bool)
        if phase is not None:
            m &= np.asarray(self.phase) == phase
        return m


# =============================================================================
# DASHBOARD
# =============================================================================
class Dashboard:
    def __init__(self, plt, fea, live: bool, total_s: float) -> None:
        af = getattr(fea, "a_f", FEA_AF)
        mf = getattr(fea, "m_f", FEA_MF)
        self.plt = plt
        self.live = live
        self.fig, (self.ax_t, self.ax_h, self.ax_f) = plt.subplots(1, 3, figsize=(20, 6.5))
        self.fig.suptitle("Phase 4 Bench Validation - Nitinol SMA Brake Duct Actuator", fontsize=14, fontweight="bold")

        ax = self.ax_t
        (self.l_temp_h,) = ax.plot([], [], color="tab:red", lw=1.8, label="Heating phase")
        (self.l_temp_c,) = ax.plot([], [], color="tab:blue", lw=1.8, label="Cooling phase")
        ax.axhline(FEA_AF, color="tab:red", ls="--", lw=1.2, label=f"Af = {FEA_AF:.0f} °C")
        ax.axhline(FEA_MF, color="tab:blue", ls="--", lw=1.2, label=f"Mf = {FEA_MF:.0f} °C")
        ax.set_xlim(0.0, total_s)
        ax.set_ylim(15.0, 105.0)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("SMA temperature [°C]")
        ax.set_title("Temperature vs time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right")

        ax = self.ax_h
        grid = np.linspace(20.0, 105.0, 341)
        ax.plot(grid, fea.stroke(grid, "heating"), color="tab:red", ls="--", lw=1.4, alpha=0.7, label="FEA heating")
        ax.plot(grid, fea.stroke(grid, "cooling"), color="tab:blue", ls="--", lw=1.4, alpha=0.7, label="FEA cooling")
        (self.l_hyst_h,) = ax.plot([], [], color="tab:red", lw=2.0, label="Bench heating")
        (self.l_hyst_c,) = ax.plot([], [], color="tab:blue", lw=2.0, label="Bench cooling")
        ax.set_xlim(20.0, 105.0)
        ax.set_ylim(-1.0, fea.stroke_max * 1.25)
        ax.set_xlabel("SMA temperature [°C]")
        ax.set_ylabel("Linkage stroke [mm]")
        ax.set_title("Stroke vs temperature (hysteresis loop)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")

        ax = self.ax_f
        lim = fea.force_max * 1.3
        ax.plot([0.0, lim], [0.0, lim], "k--", lw=1.2, label="1:1 perfect correlation")
        (self.l_force_h,) = ax.plot([], [], "o", ms=3, color="tab:red", alpha=0.55, label="Heating samples")
        (self.l_force_c,) = ax.plot([], [], "o", ms=3, color="tab:blue", alpha=0.55, label="Cooling samples")
        ax.set_xlim(-1.0, lim)
        ax.set_ylim(-1.0, lim)
        ax.set_xlabel("ANSYS predicted force [N]")
        ax.set_ylabel("Physical force [N]")
        ax.set_title("Physical vs predicted force")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")

        self.status = self.fig.text(0.01, 0.012, "", fontsize=9, family="monospace")
        self.fig.tight_layout(rect=(0.0, 0.04, 1.0, 0.95))
        if live:
            plt.ion()
            plt.show(block=False)

    def window_open(self) -> bool:
        return self.plt.fignum_exists(self.fig.number)

    def update(self, rec: Recorder, status: str) -> None:
        h, c = rec.mask("heating"), rec.mask("cooling")
        t, temp = rec.arr("t"), rec.arr("temp")
        stroke, force, fea_force = rec.arr("stroke"), rec.arr("force"), rec.arr("fea_force")
        self.l_temp_h.set_data(t[h], temp[h])
        self.l_temp_c.set_data(t[c], temp[c])
        self.l_hyst_h.set_data(temp[h], stroke[h])
        self.l_hyst_c.set_data(temp[c], stroke[c])
        self.l_force_h.set_data(fea_force[h], force[h])
        self.l_force_c.set_data(fea_force[c], force[c])
        valid = h | c
        if valid.any():
            self.ax_t.set_ylim(min(15.0, float(temp[valid].min()) - 5.0), max(105.0, float(temp[valid].max()) + 5.0))
        self.status.set_text(status)
        if self.live and self.window_open():
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def annotate(self, metrics: dict) -> None:
        box = dict(boxstyle="round", facecolor="white", edgecolor="0.6", alpha=0.9)
        self.ax_h.text(0.98, 0.04,
                       f"Hysteresis width: {fmt(metrics['hysteresis_width_C'], '.1f')} °C\n"
                       f"Onset: {fmt(metrics['onset_temp_C'], '.1f')} °C\n"
                       f"Stroke RMSE: {fmt(metrics['stroke_rmse_mm'], '.2f')} mm "
                       f"({fmt(metrics['stroke_rmse_pct'], '.1f')} %)",
                       transform=self.ax_h.transAxes, ha="right", va="bottom", fontsize=9, bbox=box)
        self.ax_f.text(0.98, 0.04,
                       f"R² = {fmt(metrics['force_r2'], '.3f')}\n"
                       f"Force RMSE: {fmt(metrics['force_rmse_N'], '.2f')} N\n"
                       f"Peak: {fmt(metrics['peak_force_N'], '.2f')} N vs {metrics['fea_force_target_N']:.1f} N",
                       transform=self.ax_f.transAxes, ha="right", va="bottom", fontsize=9, bbox=box)
        if metrics["data_source"] != "MEASURED":
            self.fig.text(0.5, 0.5, metrics["data_source"], fontsize=60, color="red", alpha=0.15,
                          ha="center", va="center", rotation=20)

    def save(self, path: str) -> None:
        self.fig.savefig(path, dpi=200)
        LOG.info("Dashboard saved: %s", path)


# =============================================================================
# TEST EXECUTION
# =============================================================================
def run_test(source, fea, args, plt):
    total_s = args.heating_s + args.cooling_s
    period = 1.0 / args.rate
    rec = Recorder()
    dashboard = Dashboard(plt, fea, live=not args.no_live, total_s=total_s)
    os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["sample_index", "host_time_s", "phase", "board_timestamp_ms", "temp_C", "force_N",
                         "stroke_mm", "fea_stroke_mm", "fea_force_N", "packet_age_ms", "stale", "source"])
        LOG.info("Logging %.0f s at %.1f Hz -> %s", total_s, args.rate, args.csv)
        t0 = time.perf_counter()
        next_tick = t0
        phase = None
        index = 0
        while True:
            now = time.perf_counter()
            t = now - t0
            if t >= total_s:
                break
            new_phase = "heating" if t < args.heating_s else "cooling"
            if new_phase != phase:
                phase = new_phase
                LOG.info("Phase -> %s at t = %.2f s", phase.upper(), t)
                command = args.heat_cmd if phase == "heating" else args.cool_cmd
                if command:
                    source.send(command)

            if source.error:
                raise RuntimeError(source.error)

            packet, age = source.snapshot()
            if packet is not None:
                stale = age > STALE_LIMIT_S
                fea_stroke = float(fea.stroke(packet.temp_c, phase)[0])
                fea_force = float(fea.force(packet.temp_c, phase)[0])
                rec.add(t, packet.temp_c, packet.force_n, packet.stroke_mm, fea_stroke, fea_force, age, phase, stale)
                writer.writerow([index, f"{t:.4f}", phase, packet.timestamp_ms, f"{packet.temp_c:.3f}",
                                 f"{packet.force_n:.3f}", f"{packet.stroke_mm:.4f}", f"{fea_stroke:.4f}",
                                 f"{fea_force:.3f}", f"{age * 1000.0:.1f}", int(stale), source.source_label])
                index += 1
                if index % CSV_FLUSH_EVERY == 0:
                    fh.flush()
                if stale and index % int(args.rate) == 0:
                    LOG.warning("Telemetry stale: last packet %.0f ms old", age * 1000.0)

            if index % DASHBOARD_EVERY == 0:
                last = rec.arr("temp")[-1] if len(rec) else float("nan")
                dashboard.update(rec, f"t = {t:6.1f} s | phase {phase:<7} | samples {len(rec):5d} | "
                                      f"packets {source.packets:6d} | rejected {source.rejected:4d} | "
                                      f"T = {last:6.2f} °C | source {source.source_label}")

            next_tick += period
            remaining = next_tick - time.perf_counter()
            if remaining <= 0.0:
                rec.missed_deadlines += 1
                next_tick = time.perf_counter()
            elif dashboard.live and dashboard.window_open():
                plt.pause(remaining)
            else:
                time.sleep(remaining)
        fh.flush()
    LOG.info("Logging complete: %d samples written", len(rec))
    return rec, dashboard


# =============================================================================
# ANALYSIS
# =============================================================================
def fmt(value, spec: str) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "n/a"
    return format(value, spec)


def smooth(values: np.ndarray, window: int = SMOOTH_WINDOW) -> np.ndarray:
    if len(values) < window:
        return values.copy()
    kernel = np.ones(window)
    return np.convolve(values, kernel, mode="same") / np.convolve(np.ones(len(values)), kernel, mode="same")


def crossing_temperature(temps: np.ndarray, strokes: np.ndarray, level: float, rising: bool) -> float:
    for i in range(1, len(strokes)):
        a, b = strokes[i - 1], strokes[i]
        if (rising and a < level <= b) or (not rising and a > level >= b):
            frac = (level - a) / (b - a) if b != a else 0.0
            return float(temps[i - 1] + frac * (temps[i] - temps[i - 1]))
    return float("nan")


def onset_temperature(temps: np.ndarray, strokes: np.ndarray, threshold: float, hold: int) -> float:
    for i in range(len(strokes) - hold + 1):
        if np.all(strokes[i:i + hold] >= threshold):
            if i == 0:
                return float(temps[0])
            return crossing_temperature(temps[:i + 1], strokes[:i + 1], threshold, rising=True)
    return float("nan")


def crank_angle_deg(stroke_mm: float) -> float:
    arg = stroke_mm / CRANK_R_MM + math.sin(CRANK_PHASE0)
    if not math.isfinite(arg) or arg > 1.0 or arg < -1.0:
        return float("nan")
    return math.degrees(math.asin(arg) - CRANK_PHASE0)


def pct_error(measured: float, reference: float) -> float:
    if not math.isfinite(measured) or not math.isfinite(reference) or reference == 0.0:
        return float("nan")
    return (measured - reference) / reference * 100.0


def fea_reference(fea, threshold: float) -> dict:
    grid = np.arange(15.0, 120.0, 0.02)
    heat = fea.stroke(grid, "heating")
    cool = fea.stroke(grid[::-1], "cooling")
    half = 0.5 * fea.stroke_max
    t50_h = crossing_temperature(grid, heat, half, rising=True)
    t50_c = crossing_temperature(grid[::-1], cool, half, rising=False)
    return {
        "onset_temp_C": onset_temperature(grid, heat, threshold, 1),
        "t50_heating_C": t50_h,
        "t50_cooling_C": t50_c,
        "hysteresis_width_C": t50_h - t50_c,
    }


def analyze(rec: Recorder, fea, source, args) -> dict:
    valid = rec.mask()
    if valid.sum() < 10:
        raise RuntimeError(f"Insufficient valid samples for analysis ({int(valid.sum())}).")
    t, temp = rec.arr("t"), rec.arr("temp")
    force, stroke = rec.arr("force"), rec.arr("stroke")
    fea_stroke, fea_force = rec.arr("fea_stroke"), rec.arr("fea_force")
    h, c = rec.mask("heating"), rec.mask("cooling")

    max_stroke = float(stroke[valid].max())
    peak_force = float(force[valid].max())
    threshold = max(ONSET_MIN_MM, ONSET_FRACTION * max_stroke)
    stroke_h = smooth(stroke[h])
    stroke_c = smooth(stroke[c])
    half = 0.5 * max_stroke
    t50_h = crossing_temperature(temp[h], stroke_h, half, rising=True) if h.any() else float("nan")
    t50_c = crossing_temperature(temp[c], stroke_c, half, rising=False) if c.any() else float("nan")
    onset = onset_temperature(temp[h], stroke_h, threshold, ONSET_HOLD_SAMPLES) if h.any() else float("nan")

    err = stroke[valid] - fea_stroke[valid]
    stroke_rmse = float(np.sqrt(np.mean(err ** 2)))
    force_err = force[valid] - fea_force[valid]
    force_rmse = float(np.sqrt(np.mean(force_err ** 2)))
    ss_res = float(np.sum(force_err ** 2))
    ss_tot = float(np.sum((force[valid] - force[valid].mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    ref = fea_reference(fea, threshold)
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    angle_meas = crank_angle_deg(max_stroke)
    angle_fea = crank_angle_deg(fea.stroke_max)
    width = t50_h - t50_c
    return {
        "data_source": source.source_label,
        "fea_model": fea.label,
        "test_date": datetime.now().isoformat(timespec="seconds"),
        "csv": os.path.abspath(args.csv),
        "samples": int(len(rec)),
        "valid_samples": int(valid.sum()),
        "stale_samples": int(len(rec) - valid.sum()),
        "effective_rate_Hz": (len(rec) - 1) / duration if duration > 0 else float("nan"),
        "missed_deadlines": int(rec.missed_deadlines),
        "packets_received": int(source.packets),
        "packets_rejected": int(source.rejected),
        "timestamp_regressions": int(source.timestamp_regressions),
        "peak_force_N": peak_force,
        "fea_force_target_N": float(fea.force_max),
        "force_error_pct": pct_error(peak_force, fea.force_max),
        "max_stroke_mm": max_stroke,
        "fea_stroke_max_mm": float(fea.stroke_max),
        "stroke_error_pct": pct_error(max_stroke, fea.stroke_max),
        "opening_angle_deg": angle_meas,
        "fea_opening_angle_deg": angle_fea,
        "angle_error_pct": pct_error(angle_meas, angle_fea),
        "onset_threshold_mm": threshold,
        "onset_temp_C": onset,
        "fea_onset_temp_C": ref["onset_temp_C"],
        "onset_error_pct": pct_error(onset, ref["onset_temp_C"]),
        "t50_heating_C": t50_h,
        "t50_cooling_C": t50_c,
        "hysteresis_width_C": width,
        "fea_hysteresis_width_C": ref["hysteresis_width_C"],
        "hysteresis_error_pct": pct_error(width, ref["hysteresis_width_C"]),
        "stroke_rmse_mm": stroke_rmse,
        "stroke_rmse_pct": stroke_rmse / fea.stroke_max * 100.0,
        "force_rmse_N": force_rmse,
        "force_r2": r2,
        "peak_temp_C": float(temp[valid].max()),
        "min_temp_C": float(temp[valid].min()),
    }


def print_table(title: str, headers, rows) -> None:
    widths = [max([len(str(h))] + [len(str(r[i])) for r in rows]) for i, h in enumerate(headers)]
    border = "+" + "+".join("-" * (w + 2) for w in widths) + "+"
    print(f"\n{title}\n{border}")
    print("| " + " | ".join(str(h).ljust(w) for h, w in zip(headers, widths)) + " |")
    print(border)
    for row in rows:
        print("| " + " | ".join(str(c).ljust(w) for c, w in zip(row, widths)) + " |")
    print(border)


def print_summary(m: dict) -> None:
    print("\n" + "=" * 100)
    print(" PHASE 4 BENCH VALIDATION - PHYSICAL vs ANSYS CORRELATION")
    print(f" Data source: {m['data_source']}   |   FEA reference: {m['fea_model']}")
    print("=" * 100)
    print_table("Correlation summary", ("Metric", "Physical", "ANSYS / FEA", "Error"), [
        ("Peak actuation force", f"{fmt(m['peak_force_N'], '.2f')} N", f"{m['fea_force_target_N']:.2f} N",
         f"{fmt(m['force_error_pct'], '+.1f')} %"),
        ("Maximum louver stroke", f"{fmt(m['max_stroke_mm'], '.2f')} mm", f"{m['fea_stroke_max_mm']:.2f} mm",
         f"{fmt(m['stroke_error_pct'], '+.1f')} %"),
        ("Louver opening angle (crank kinematics)", f"{fmt(m['opening_angle_deg'], '.1f')} °",
         f"{fmt(m['fea_opening_angle_deg'], '.1f')} °", f"{fmt(m['angle_error_pct'], '+.1f')} %"),
        ("Thermal trigger onset temperature", f"{fmt(m['onset_temp_C'], '.1f')} °C",
         f"{fmt(m['fea_onset_temp_C'], '.1f')} °C", f"{fmt(m['onset_error_pct'], '+.1f')} %"),
        ("Hysteresis width (50 % stroke)", f"{fmt(m['hysteresis_width_C'], '.1f')} °C",
         f"{fmt(m['fea_hysteresis_width_C'], '.1f')} °C", f"{fmt(m['hysteresis_error_pct'], '+.1f')} %"),
        ("Stroke RMSE (physical vs FEA)", f"{fmt(m['stroke_rmse_mm'], '.3f')} mm", "-",
         f"{fmt(m['stroke_rmse_pct'], '.1f')} % of FEA max"),
        ("Force RMSE / R²", f"{fmt(m['force_rmse_N'], '.2f')} N", "-", f"R² = {fmt(m['force_r2'], '.3f')}"),
        ("Overall model correlation error (stroke NRMSE)", "-", "-", f"{fmt(m['stroke_rmse_pct'], '.1f')} %"),
    ])
    print_table("Acquisition health", ("Item", "Value"), [
        ("Samples logged / valid / stale", f"{m['samples']} / {m['valid_samples']} / {m['stale_samples']}"),
        ("Effective sample rate", f"{fmt(m['effective_rate_Hz'], '.2f')} Hz (target {SAMPLE_HZ:.0f} Hz)"),
        ("Missed loop deadlines", str(m["missed_deadlines"])),
        ("Packets received / rejected", f"{m['packets_received']} / {m['packets_rejected']}"),
        ("Board timestamp regressions (resets)", str(m["timestamp_regressions"])),
        ("Temperature range", f"{fmt(m['min_temp_C'], '.1f')} - {fmt(m['peak_temp_C'], '.1f')} °C"),
        ("CSV log", m["csv"]),
    ])
    if m["data_source"] != "MEASURED":
        print(f"\n  WARNING: data source is {m['data_source']} - this is not physical validation.")
    print("=" * 100)


# =============================================================================
# MAIN
# =============================================================================
def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Phase 4 Nitinol brake duct bench DAQ and FEA correlation.")
    parser.add_argument("--port", help="Serial port (e.g. COM5). Auto-detected when omitted.")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--rate", type=float, default=SAMPLE_HZ, help="Logging rate [Hz]")
    parser.add_argument("--heating-s", type=float, default=HEATING_S)
    parser.add_argument("--cooling-s", type=float, default=COOLING_S)
    parser.add_argument("--csv", default=CSV_PATH)
    parser.add_argument("--dashboard", default=DASHBOARD_PATH)
    parser.add_argument("--summary", default=SUMMARY_PATH)
    parser.add_argument("--fea-csv", help="Optional tabulated FEA prediction CSV")
    parser.add_argument("--heat-cmd", help="Optional line sent to the DAQ at heating start")
    parser.add_argument("--cool-cmd", help="Optional line sent to the DAQ at cooling start")
    parser.add_argument("--connect-timeout", type=float, default=8.0)
    parser.add_argument("--no-live", action="store_true", help="Disable the live GUI dashboard")
    parser.add_argument("--simulate", action="store_true", help="Hardware-free dry run (SIMULATED data)")
    parser.add_argument("--source-label", default="MEASURED",
                        help="Label for serial data, e.g. 'DIGITAL TWIN' when the port is the rig twin")
    parser.add_argument("--as-c", type=float, default=FEA_AS, help="FEA reference austenite start [C]")
    parser.add_argument("--af-c", type=float, default=FEA_AF, help="FEA reference austenite finish [C]")
    parser.add_argument("--ms-c", type=float, default=FEA_MS, help="FEA reference martensite start [C]")
    parser.add_argument("--mf-c", type=float, default=FEA_MF, help="FEA reference martensite finish [C]")
    parser.add_argument("--stroke-max", type=float, default=FEA_STROKE_MAX_MM, help="FEA hot stroke [mm]")
    parser.add_argument("--force-max", type=float, default=FEA_FORCE_MAX_N, help="FEA force at full stroke [N]")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)-7s  %(message)s", datefmt="%H:%M:%S")
    if args.no_live:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    try:
        fea = TabulatedFEA(args.fea_csv) if args.fea_csv else LiangRogersFEA(args.as_c, args.af_c, args.ms_c, args.mf_c, args.stroke_max, args.force_max)
    except (OSError, ValueError) as exc:
        LOG.error("FEA reference could not be loaded: %s", exc)
        return 1
    LOG.info("FEA reference: %s (stroke_max %.2f mm, force_max %.2f N)", fea.label, fea.stroke_max, fea.force_max)

    try:
        if args.simulate:
            source = SimulatedTelemetry(args.heating_s)
        else:
            port = args.port or autodetect_port(args.baud)
            source = SerialTelemetry(port, args.baud, args.source_label)
        source.start()
    except Exception as exc:  # noqa: BLE001 - connection failures end the run cleanly
        LOG.error("Could not start telemetry: %s", exc)
        return 1

    rec = dashboard = None
    try:
        if not source.wait_first(args.connect_timeout):
            raise RuntimeError(source.error or f"No valid telemetry within {args.connect_timeout:.0f} s.")
        rec, dashboard = run_test(source, fea, args, plt)
    except KeyboardInterrupt:
        LOG.warning("Test interrupted by operator - analysing partial data.")
    except Exception as exc:  # noqa: BLE001
        LOG.error("Acquisition failed: %s", exc)
    finally:
        source.stop()

    if rec is None or len(rec) == 0:
        LOG.error("No data recorded; no report generated.")
        return 1
    try:
        metrics = analyze(rec, fea, source, args)
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 1

    dashboard.update(rec, f"COMPLETE | samples {len(rec)} | source {metrics['data_source']} | {fea.label}")
    dashboard.annotate(metrics)
    os.makedirs(os.path.dirname(args.dashboard) or ".", exist_ok=True)
    dashboard.save(args.dashboard)
    with open(args.summary, "w", encoding="utf-8") as fh:
        json.dump({k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in metrics.items()},
                  fh, indent=2)
    LOG.info("Correlation summary written: %s", args.summary)
    print_summary(metrics)

    if not args.no_live and dashboard.window_open():
        LOG.info("Close the dashboard window to exit.")
        plt.ioff()
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
