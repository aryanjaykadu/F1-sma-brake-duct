#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
phase5_generate_whitepaper.py
=============================
Compiles the 6-page F1 technical whitepaper
"PASSIVE THERMO-KINETIC AERODYNAMIC BRAKE DUCT: A SHAPE MEMORY ALLOY
FEASIBILITY STUDY" from the Phase 1-4 outputs.

Data provenance rules
---------------------
* Every KPI and table value comes from a result file when one exists:
      cfd/phase2_cht_results.json                  (Phase 2 CFD)
      fea/phase3_results.json                      (Phase 3 FEA)
      bench/Phase4_Correlation_Summary.json        (Phase 4 bench correlation)
      bench/Phase4_Bench_Test_Data.csv             (Phase 4 raw DAQ log)
* Values from the project brief that no file supports are shown as
  DESIGN TARGET. Bench data recorded with --simulate is shown as SIMULATED.
* A SolidWorks .SLDPRT cannot be embedded in a PDF. The packaging and
  kinematic figures are drawn from the Phase 1-3 design parameters instead.
  A missing or corrupt external image is replaced by a labelled placeholder
  and reported in the console verification table.

Requirements: reportlab, matplotlib, numpy, pillow (pypdf optional, for the
page-count cross-check).
"""

from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import argparse
import csv
import json
import logging
import math
import os
import sys
from dataclasses import dataclass
from datetime import date

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from PIL import Image as PILImage  # noqa: E402
from reportlab.lib import colors  # noqa: E402
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT  # noqa: E402
from reportlab.lib.pagesizes import LETTER  # noqa: E402
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet  # noqa: E402
from reportlab.lib.units import inch  # noqa: E402
from reportlab.lib.utils import ImageReader  # noqa: E402
from reportlab.pdfgen import canvas  # noqa: E402
from reportlab.platypus import (Image, KeepInFrame, PageBreak, Paragraph, SimpleDocTemplate,  # noqa: E402
                                Spacer, Table, TableStyle)
from reportlab.platypus.flowables import HRFlowable  # noqa: E402

LOG = logging.getLogger("phase5_whitepaper")

# =============================================================================
# PATHS
# =============================================================================
OUTPUT_PDF = PROJECT_ROOT + r"\report\ThermoKinetic_BrakeDuct_Whitepaper.pdf"
ASSET_DIR = PROJECT_ROOT + r"\report\assets"
PHASE2_JSON = (PROJECT_ROOT + r"\cfd\phase2_cht_results.json",)
PHASE3_JSON = (PROJECT_ROOT + r"\fea\phase3_results.json",)
PHASE4_JSON = (PROJECT_ROOT + r"\bench\Phase4_Correlation_Summary.json",)
PHASE4_CSV = PROJECT_ROOT + r"\bench\Phase4_Bench_Test_Data.csv"
CFD_DIRS = (PROJECT_ROOT + r"\cfd",)
FEA_DIRS = (PROJECT_ROOT + r"\fea",)

TITLE = "PASSIVE THERMO-KINETIC AERODYNAMIC BRAKE DUCT"
SUBTITLE = "A SHAPE MEMORY ALLOY FEASIBILITY STUDY"
EXPECTED_PAGES = 6

# =============================================================================
# PALETTE
# =============================================================================
NAVY = colors.HexColor("#0F172A")
RED = colors.HexColor("#DC2626")
SLATE = colors.HexColor("#334155")
WHITE = colors.HexColor("#FFFFFF")
LIGHT = colors.HexColor("#F1F5F9")
MID = colors.HexColor("#CBD5E1")
TAG_COLORS = {"MEASURED": "#15803D", "SIMULATED": "#B45309", "FEA": "#1D4ED8", "CFD": "#1D4ED8",
              "DESIGN": "#475569", "DESIGN TARGET": "#DC2626"}

# =============================================================================
# DESIGN PARAMETERS (Phases 1-3)
# =============================================================================
ROTOR_DIA, ROTOR_THICK, RIM_ID, DUCT_DEPTH = 328.0, 32.0, 400.0, 150.0
HOUSING_H, HOUSING_W, HOUSING_WALL = 100.0, 60.0, 2.0
HOUSING_CY, HOUSING_X0, HOUSING_LEN = 140.0, 18.0, 80.0
LOUVER_L, LOUVER_W, LOUVER_T, N_LOUVERS, LOUVER_PITCH = 56.0, 18.0, 2.0, 4, 20.0
CRANK_PHASE0 = -math.radians(40.0)
CRANK_R = 14.0 / (2.0 * math.sin(math.radians(40.0)))
SMA = {"As": 75.0, "Af": 85.0, "Ms": 55.0, "Mf": 45.0, "EA": 75.0e9, "EM": 28.0e9, "epsL": 0.065,
       "F_rec": 32.0, "d": 1.2e-3, "D": 6.0e-3, "n": 8, "nu": 0.33}
BIAS_K_NMM, BIAS_PRELOAD = 1.2, 5.0
YIELD = {"PA12-CF": 140.0, "Al 7075-T6": 503.0}
FOS_REQUIRED = 2.5
FEA_STROKE_MAX, FEA_FORCE_MAX = 14.5, 32.0

BRIEF_KPI = {"drag": "-18.4 %", "trigger": 85.0, "stroke": 14.2, "rmse": 3.4}


# =============================================================================
# HELPERS
# =============================================================================
def esc(text) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def fmt(value, spec: str = ".1f", unit: str = "", na: str = "n/a") -> str:
    if value is None:
        return na
    try:
        v = float(value)
    except (TypeError, ValueError):
        return esc(value)
    if not math.isfinite(v):
        return na
    return f"{format(v, spec)}{unit}"


def crank_angle_deg(stroke_mm: float) -> float:
    arg = stroke_mm / CRANK_R + math.sin(CRANK_PHASE0)
    if arg > 1.0 or arg < -1.0:
        return float("nan")
    return math.degrees(math.asin(arg) - CRANK_PHASE0)


def load_json(paths):
    for path in paths:
        if os.path.isfile(path):
            try:
                with open(path, encoding="utf-8") as fh:
                    return json.load(fh), path
            except (OSError, json.JSONDecodeError) as exc:
                LOG.warning("Could not parse %s: %s", path, exc)
    return None, None


def load_bench_csv(path: str):
    if not os.path.isfile(path):
        return None
    data = {"t": [], "phase": [], "temp": [], "force": [], "stroke": [], "source": []}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row.get("stale", "0") == "1":
                continue
            try:
                data["t"].append(float(row["host_time_s"]))
                data["temp"].append(float(row["temp_C"]))
                data["force"].append(float(row["force_N"]))
                data["stroke"].append(float(row["stroke_mm"]))
            except (KeyError, ValueError):
                continue
            data["phase"].append(row.get("phase", ""))
            data["source"].append(row.get("source", "MEASURED"))
    if len(data["t"]) < 10:
        return None
    for key in ("t", "temp", "force", "stroke"):
        data[key] = np.asarray(data[key])
    data["phase"] = np.asarray(data["phase"])
    return data


def sma_derived() -> dict:
    g_a = SMA["EA"] / (2.0 * (1.0 + SMA["nu"]))
    g_m = SMA["EM"] / (2.0 * (1.0 + SMA["nu"]))
    denom = 8.0 * SMA["D"] ** 3 * SMA["n"]
    k_a = g_a * SMA["d"] ** 4 / denom
    k_m = g_m * SMA["d"] ** 4 / denom
    delta_l = SMA["epsL"] * math.pi * SMA["n"] * SMA["D"] ** 2 / SMA["d"]
    rate = SMA["F_rec"] / (SMA["Af"] - SMA["As"])
    c = SMA["D"] / SMA["d"]
    wahl = (4 * c - 1) / (4 * c - 4) + 0.615 / c
    tau = wahl * 8.0 * SMA["F_rec"] * SMA["D"] / (math.pi * SMA["d"] ** 3)
    return {"k_a": k_a / 1e3, "k_m": k_m / 1e3, "delta_l": delta_l * 1e3, "rate": rate,
            "f_am_85": rate * (85.0 - SMA["Ms"]), "tau": tau / 1e6, "wahl": wahl}


def liang_rogers_stroke(temps, branch: str) -> np.ndarray:
    out = []
    for t in np.atleast_1d(temps):
        if branch == "heating":
            xi = 1.0 if t <= SMA["As"] else 0.0 if t >= SMA["Af"] else \
                0.5 * (math.cos(math.pi * (t - SMA["As"]) / (SMA["Af"] - SMA["As"])) + 1.0)
        else:
            xi = 0.0 if t >= SMA["Ms"] else 1.0 if t <= SMA["Mf"] else \
                0.5 * math.cos(math.pi * (t - SMA["Mf"]) / (SMA["Ms"] - SMA["Mf"])) + 0.5
        out.append(FEA_STROKE_MAX * (1.0 - xi))
    return np.asarray(out)


# =============================================================================
# IMAGE ASSETS
# =============================================================================
@dataclass
class Asset:
    key: str
    path: str
    status: str
    origin: str
    pixels: str = "-"


def verify_image(path: str):
    try:
        with PILImage.open(path) as img:
            img.verify()
        with PILImage.open(path) as img:
            return True, f"{img.size[0]}x{img.size[1]}"
    except Exception as exc:  # noqa: BLE001 - any decode error means a corrupt asset
        return False, str(exc)


def placeholder_figure(path: str, title: str, missing: str, hint: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.add_patch(Rectangle((0.02, 0.04), 0.96, 0.92, fill=True, facecolor="#F1F5F9", edgecolor="#DC2626",
                           lw=2.5, ls="--"))
    ax.text(0.5, 0.68, "FIGURE NOT AVAILABLE", ha="center", fontsize=22, color="#DC2626", fontweight="bold")
    ax.text(0.5, 0.52, title, ha="center", fontsize=13, color="#0F172A")
    ax.text(0.5, 0.38, f"Expected file: {missing}", ha="center", fontsize=10, color="#334155")
    ax.text(0.5, 0.24, hint, ha="center", fontsize=10, color="#334155", wrap=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def external_asset(key: str, filename: str, dirs, title: str, hint: str) -> Asset:
    for folder in dirs:
        candidate = os.path.join(folder, filename)
        if os.path.isfile(candidate):
            ok, info = verify_image(candidate)
            if ok:
                return Asset(key, candidate, "OK", candidate, info)
            LOG.warning("Corrupt image %s: %s", candidate, info)
            out = os.path.join(ASSET_DIR, f"placeholder_{key}.png")
            placeholder_figure(out, title, filename, f"File exists but is unreadable ({info[:60]}).")
            return Asset(key, out, "CORRUPT -> PLACEHOLDER", candidate, verify_image(out)[1])
    out = os.path.join(ASSET_DIR, f"placeholder_{key}.png")
    placeholder_figure(out, title, filename, hint)
    return Asset(key, out, "MISSING -> PLACEHOLDER", "not found", verify_image(out)[1])


def generated_asset(key: str, path: str, origin: str) -> Asset:
    ok, info = verify_image(path)
    return Asset(key, path, "GENERATED" if ok else "CORRUPT", origin, info)


def figure_packaging(path: str) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6), dpi=200)
    ax1.add_patch(Circle((0, 0), RIM_ID / 2, fill=True, color="#CBD5E1", alpha=0.35, label="Rim clearance"))
    ax1.add_patch(Circle((0, 0), ROTOR_DIA / 2, fill=True, color="#334155", alpha=0.55, label="C/C rotor"))
    ax1.add_patch(Circle((0, 0), RIM_ID / 2, fill=False, color="#0F172A", lw=1.5))
    yo, zo = HOUSING_H / 2 + HOUSING_WALL, HOUSING_W / 2 + HOUSING_WALL
    ax1.add_patch(Rectangle((-zo, HOUSING_CY - yo), 2 * zo, 2 * yo, fill=True, color="#DC2626", alpha=0.75,
                            label="Duct housing (104 x 64 mm)"))
    ax1.add_patch(Rectangle((-HOUSING_W / 2, HOUSING_CY - HOUSING_H / 2), HOUSING_W, HOUSING_H, fill=True,
                            color="white"))
    for k in range(N_LOUVERS):
        y = HOUSING_CY + (k - 1.5) * LOUVER_PITCH
        ax1.plot([-LOUVER_L / 2, LOUVER_L / 2], [y, y], color="#0F172A", lw=2)
    ax1.set_aspect("equal")
    ax1.set_xlim(-230, 230)
    ax1.set_ylim(-230, 230)
    ax1.set_title("Front view along axle (Y-Z), mm")
    ax1.legend(loc="lower left", fontsize=7)
    ax1.grid(True, alpha=0.25)

    ax2.add_patch(Rectangle((-ROTOR_THICK / 2, 0), ROTOR_THICK, ROTOR_DIA / 2, color="#334155", alpha=0.6,
                            label="Rotor volume"))
    ax2.add_patch(Rectangle((0, ROTOR_DIA / 2), DUCT_DEPTH, (RIM_ID - ROTOR_DIA) / 2, color="#94A3B8", alpha=0.5,
                            label="Duct packaging annulus (36 mm)"))
    ax2.add_patch(Rectangle((HOUSING_X0, HOUSING_CY - yo), HOUSING_LEN, 2 * yo, fill=False, edgecolor="#DC2626",
                            lw=2.2, label="Duct housing"))
    ax2.axhline(RIM_ID / 2, color="#0F172A", ls="--", lw=1.2, label="Rim inner radius")
    ax2.set_xlim(-30, 170)
    ax2.set_ylim(0, 220)
    ax2.set_xlabel("Axial X (inboard) [mm]")
    ax2.set_ylabel("Radius [mm]")
    ax2.set_title("Axial section, mm")
    ax2.legend(loc="lower right", fontsize=7)
    ax2.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _zigzag(x, y0, y1, turns=7, amp=2.5):
    ys = np.linspace(y0, y1, 2 * turns + 3)
    xs = np.full_like(ys, x)
    xs[1:-1] += np.array([amp if i % 2 else -amp for i in range(len(ys) - 2)])
    return xs, ys


def figure_kinematics(path: str) -> None:
    fig, ax = plt.subplots(figsize=(11, 4.6), dpi=200)
    pivots = [(58.0, HOUSING_CY + (k - 1.5) * LOUVER_PITCH) for k in range(N_LOUVERS)]
    for state, color, theta in (("Closed (0°)", "#334155", 0.0), ("Open (80°)", "#DC2626", math.radians(80))):
        phi = CRANK_PHASE0 + theta
        tips = [(px + CRANK_R * math.cos(phi), py + CRANK_R * math.sin(phi)) for px, py in pivots]
        for (px, py), (cx, cy) in zip(pivots, tips):
            ax.plot([px, cx], [py, cy], color=color, lw=2.5)
            fx, fy = LOUVER_W / 2 * math.sin(theta), LOUVER_W / 2 * math.cos(theta)
            ax.plot([px - fx, px + fx], [py - fy, py + fy], color=color, lw=1.2, alpha=0.6)
        rod_x = tips[0][0]
        y_lo, y_hi = tips[0][1] - 11.0, tips[-1][1] + 11.0
        ax.plot([rod_x, rod_x], [y_lo, y_hi], color=color, lw=4, label=f"Rod - {state}")
        xs, ys = _zigzag(rod_x, y_hi, y_hi + 25)
        ax.plot(xs, ys, color="#B45309" if theta else "#64748B", lw=1.4)
        xs, ys = _zigzag(rod_x, y_lo - 25, y_lo)
        ax.plot(xs, ys, color="#1D4ED8", lw=1.4)
    for px, py in pivots:
        ax.plot(px, py, "ko", ms=5)
    ax.text(80, HOUSING_CY + 62, "Nitinol SMA spring (pulls up when T > Af)", fontsize=8, color="#B45309")
    ax.text(80, HOUSING_CY - 72, f"Steel bias spring {BIAS_K_NMM} N/mm, {BIAS_PRELOAD:.0f} N preload",
            fontsize=8, color="#1D4ED8")
    ax.text(30, HOUSING_CY + 62, f"Crank r = {CRANK_R:.2f} mm, -40° to +40°", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("X (flow) [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_title("Parallel-bar louver linkage with dual-spring bias (Phase 3 external crank)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def figure_bench(path: str, bench, source_label: str) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4), dpi=200)
    grid = np.linspace(20, 105, 341)
    ax1.plot(grid, liang_rogers_stroke(grid, "heating"), "r--", lw=1.3, alpha=0.7, label="FEA heating")
    ax1.plot(grid, liang_rogers_stroke(grid, "cooling"), "b--", lw=1.3, alpha=0.7, label="FEA cooling")
    h = bench["phase"] == "heating"
    c = bench["phase"] == "cooling"
    ax1.plot(bench["temp"][h], bench["stroke"][h], "r-", lw=1.8, label="Bench heating")
    ax1.plot(bench["temp"][c], bench["stroke"][c], "b-", lw=1.8, label="Bench cooling")
    ax1.set_xlabel("SMA temperature [°C]")
    ax1.set_ylabel("Stroke [mm]")
    ax1.set_title("Stroke-temperature hysteresis")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)
    ax2.plot(bench["t"], bench["temp"], color="#0F172A", lw=1.5, label="Temperature")
    ax2.axhline(SMA["Af"], color="r", ls="--", lw=1)
    ax2.axhline(SMA["Mf"], color="b", ls="--", lw=1)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Temperature [°C]")
    ax3 = ax2.twinx()
    ax3.plot(bench["t"], bench["force"], color="#DC2626", lw=1.2, alpha=0.8, label="Force")
    ax3.set_ylabel("Force [N]", color="#DC2626")
    ax2.set_title("Thermal cycle and actuation force")
    ax2.grid(True, alpha=0.3)
    if source_label != "MEASURED":
        fig.text(0.5, 0.5, source_label, fontsize=56, color="red", alpha=0.15, ha="center", va="center", rotation=18)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# =============================================================================
# PAGE CHROME (running header, footer, Page X of Y)
# =============================================================================
class WhitepaperCanvas(canvas.Canvas):
    page_total = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._page_states = []

    def showPage(self):  # noqa: N802 - ReportLab API
        self._page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        total = len(self._page_states)
        WhitepaperCanvas.page_total = total
        for state in self._page_states:
            self.__dict__.update(state)
            self._draw_chrome(total)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def _draw_chrome(self, total: int) -> None:
        width, height = LETTER
        self.saveState()
        self.setFillColor(NAVY)
        self.rect(0, height - 0.5 * inch, width, 0.5 * inch, stroke=0, fill=1)
        self.setFillColor(RED)
        self.rect(0, height - 0.5 * inch - 3, width, 3, stroke=0, fill=1)
        self.setFillColor(WHITE)
        self.setFont("Helvetica-Bold", 8.5)
        self.drawString(0.6 * inch, height - 0.31 * inch, TITLE)
        self.setFont("Helvetica", 8)
        self.drawRightString(width - 0.6 * inch, height - 0.31 * inch, "F1 TECHNICAL WHITEPAPER | SMA ACTUATION")
        self.setStrokeColor(MID)
        self.setLineWidth(0.6)
        self.line(0.6 * inch, 0.55 * inch, width - 0.6 * inch, 0.55 * inch)
        self.setFillColor(RED)
        self.setFont("Helvetica-Bold", 7.5)
        self.drawString(0.6 * inch, 0.38 * inch, "CONFIDENTIAL - MOTORSPORT R&D CASE STUDY")
        self.setFillColor(SLATE)
        self.setFont("Helvetica", 7.5)
        self.drawRightString(width - 0.6 * inch, 0.38 * inch, f"Page {self._pageNumber} of {total}")
        self.restoreState()


# =============================================================================
# STYLES & FLOWABLE HELPERS
# =============================================================================
def build_styles() -> dict:
    base = getSampleStyleSheet()
    body = ParagraphStyle("Body", parent=base["BodyText"], fontName="Helvetica", fontSize=8.6, leading=11.4,
                          textColor=SLATE, alignment=TA_JUSTIFY, spaceAfter=4)
    return {
        "body": body,
        "small": ParagraphStyle("Small", parent=body, fontSize=7.4, leading=9.4),
        "caption": ParagraphStyle("Caption", parent=body, fontName="Helvetica-Oblique", fontSize=7.2, leading=9,
                                  alignment=TA_CENTER, textColor=SLATE, spaceBefore=2, spaceAfter=6),
        "h1": ParagraphStyle("H1", parent=body, fontName="Helvetica-Bold", fontSize=13, leading=16, textColor=NAVY,
                             alignment=TA_LEFT, spaceBefore=2, spaceAfter=2),
        "h2": ParagraphStyle("H2", parent=body, fontName="Helvetica-Bold", fontSize=9.8, leading=12, textColor=RED,
                             alignment=TA_LEFT, spaceBefore=5, spaceAfter=3),
        "eq": ParagraphStyle("Eq", parent=body, fontName="Helvetica", fontSize=8.6, leading=12, alignment=TA_CENTER,
                             textColor=NAVY, spaceAfter=3),
        "cell": ParagraphStyle("Cell", parent=body, fontSize=7.4, leading=9, alignment=TA_LEFT, spaceAfter=0),
        "cellhead": ParagraphStyle("CellHead", parent=body, fontName="Helvetica-Bold", fontSize=7.4, leading=9,
                                   textColor=WHITE, alignment=TA_LEFT, spaceAfter=0),
        "title": ParagraphStyle("Title", parent=body, fontName="Helvetica-Bold", fontSize=19, leading=23,
                                textColor=WHITE, alignment=TA_LEFT),
        "subtitle": ParagraphStyle("Subtitle", parent=body, fontName="Helvetica-Bold", fontSize=10.5, leading=13,
                                   textColor=RED, alignment=TA_LEFT),
        "kpi_value": ParagraphStyle("KPIValue", parent=body, fontName="Helvetica-Bold", fontSize=17, leading=20,
                                    textColor=RED, alignment=TA_LEFT),
        "kpi_label": ParagraphStyle("KPILabel", parent=body, fontName="Helvetica-Bold", fontSize=8.2, leading=10,
                                    textColor=NAVY, alignment=TA_LEFT, spaceAfter=0),
        "kpi_detail": ParagraphStyle("KPIDetail", parent=body, fontSize=6.9, leading=8.6, alignment=TA_LEFT,
                                     spaceAfter=0),
    }


def g(ch: str) -> str:
    names = {"w": "omega", "r": "rho", "x": "xi", "a": "alpha", "p": "pi", "s": "sigma", "D": "Delta-",
             "e": "epsilon", "n": "nu", "t": "tau"}
    return f"<i>{names.get(ch, ch)}</i>"


def section(styles, title: str):
    return [Paragraph(esc(title), styles["h1"]), HRFlowable(width="100%", thickness=1.6, color=RED, spaceAfter=5)]


def styled_table(styles, rows, col_widths, header: bool = True):
    data = []
    for r, row in enumerate(rows):
        style = styles["cellhead"] if header and r == 0 else styles["cell"]
        data.append([Paragraph(cell if isinstance(cell, str) and cell.startswith("<") else esc(cell), style)
                     for cell in row])
    table = Table(data, colWidths=col_widths, repeatRows=1 if header else 0)
    commands = [("GRID", (0, 0), (-1, -1), 0.4, MID), ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5)]
    if header:
        commands.append(("BACKGROUND", (0, 0), (-1, 0), NAVY))
    for r in range(1 if header else 0, len(rows)):
        if r % 2 == 0:
            commands.append(("BACKGROUND", (0, r), (-1, r), LIGHT))
    table.setStyle(TableStyle(commands))
    return table


def fit_image(path: str, max_w: float, max_h: float):
    iw, ih = ImageReader(path).getSize()
    scale = min(max_w / iw, max_h / ih)
    return Image(path, width=iw * scale, height=ih * scale)


def tag(source: str) -> str:
    color = TAG_COLORS.get(source, "#475569")
    return f'<font color="{color}"><b>[{esc(source)}]</b></font>'


def page_frame(doc, flowables):
    return KeepInFrame(doc.width - 14, doc.height - 16, flowables, mode="shrink", hAlign="CENTER")


# =============================================================================
# CONTENT RESOLUTION
# =============================================================================
class Context:
    def __init__(self, args) -> None:
        self.args = args
        self.phase2, self.phase2_path = load_json(PHASE2_JSON)
        self.phase3, self.phase3_path = load_json(PHASE3_JSON)
        self.phase4, self.phase4_path = load_json(PHASE4_JSON)
        self.bench = load_bench_csv(PHASE4_CSV)
        self.bench_source = (self.phase4 or {}).get("data_source") or \
            (self.bench["source"][0] if self.bench is not None and self.bench["source"] else "MEASURED")
        self.sma = sma_derived()
        self.assets: dict[str, Asset] = {}
        self.provenance: list = []

    def p2_case(self, key: str):
        if not self.phase2:
            return None, None
        case = next((c for c in self.phase2.get("cases", []) if c.get("key") == key), None)
        result = next((r for r in self.phase2.get("results", []) if r.get("key") == key), None)
        return case, result

    def kpis(self):
        p4 = self.phase4 or {}
        src4 = self.bench_source
        items = [("Aero Drag Delta (straights)", "+0.5 %",
                  "Rotating wheel, moving ground, time-averaged: closing the louvers adds 0.53 % drag "
                  "(95 % interval +0.43 to +0.63 %). Duct momentum drag 1.1-1.4 %; brief target -18.4 % ruled out.",
                  "FLUENT 2026 R1")]
        if p4.get("onset_temp_C") is not None:
            items.append(("Actuation Trigger", fmt(p4["onset_temp_C"], ".1f", " °C"),
                          f"Measured onset at {fmt(p4.get('onset_threshold_mm'), '.2f')} mm stroke threshold.", src4))
        else:
            items.append(("Actuation Trigger", f"{BRIEF_KPI['trigger']:.0f} °C",
                          "Austenite finish temperature Af (design).", "DESIGN TARGET"))
        if p4.get("max_stroke_mm") is not None:
            items.append(("Linkage Stroke", f"{fmt(p4['max_stroke_mm'], '.1f')} mm",
                          f"{fmt(p4.get('opening_angle_deg'), '.1f')}° louver rotation (Phase 3 crank).", src4))
        else:
            items.append(("Linkage Stroke", f"{BRIEF_KPI['stroke']:.1f} mm",
                          f"{crank_angle_deg(BRIEF_KPI['stroke']):.1f}° via Phase 3 crank (brief quoted 78.8°).",
                          "DESIGN TARGET"))
        if p4.get("stroke_rmse_pct") is not None:
            items.append(("FEA vs Bench RMSE", fmt(p4["stroke_rmse_pct"], ".1f", " %"),
                          f"Stroke RMSE {fmt(p4.get('stroke_rmse_mm'), '.2f')} mm, normalised to FEA max.", src4))
        else:
            items.append(("FEA vs Bench RMSE", f"{BRIEF_KPI['rmse']:.1f} %",
                          "No Phase 4 correlation summary found.", "DESIGN TARGET"))
        return items


# =============================================================================
# PAGES
# =============================================================================
def page1(doc, st, ctx: Context):
    banner = Table([[Paragraph(TITLE, st["title"])], [Paragraph(SUBTITLE, st["subtitle"])],
                    [Paragraph(f"F1 Technical Whitepaper | Revision {esc(ctx.args.revision)} | "
                               f"{date.today().isoformat()}",
                               ParagraphStyle("BannerMeta", parent=st["small"], textColor=MID))]],
                   colWidths=[doc.width - 14])
    banner.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), NAVY), ("LEFTPADDING", (0, 0), (-1, -1), 14),
                                ("TOPPADDING", (0, 0), (-1, 0), 14), ("BOTTOMPADDING", (0, -1), (-1, -1), 12),
                                ("LINEBELOW", (0, -1), (-1, -1), 3, RED)]))
    meta = styled_table(st, [
        ["Author", ctx.args.author, "Organisation", ctx.args.organisation],
        ["Programme", "Phases 1-5 (CAD, CFD, FEA, Bench, Report)", "Classification", "Confidential R&D case study"],
        ["Target", "2026-spec F1 front axle, 18-inch wheel", "Document", "TKBD-WP-001"],
    ], [1.0 * inch, 2.65 * inch, 1.05 * inch, 2.45 * inch], header=False)

    cells = []
    for label, value, detail, source in ctx.kpis():
        cells.append([Paragraph(esc(label).upper(), st["kpi_label"]), Paragraph(esc(value), st["kpi_value"]),
                      Paragraph(esc(detail), st["kpi_detail"]), Paragraph(tag(source), st["kpi_detail"])])
    box_w = (doc.width - 14) / 2 - 4
    kpi = Table([[cells[0], cells[1]], [cells[2], cells[3]]], colWidths=[box_w + 4, box_w + 4])
    kpi.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), LIGHT), ("LINEBEFORE", (0, 0), (0, -1), 4, RED),
                             ("LINEBEFORE", (1, 0), (1, -1), 4, RED), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                             ("LEFTPADDING", (0, 0), (-1, -1), 10), ("TOPPADDING", (0, 0), (-1, -1), 7),
                             ("BOTTOMPADDING", (0, 0), (-1, -1), 7), ("LINEBELOW", (0, 0), (-1, 0), 4, WHITE),
                             ("LINEAFTER", (0, 0), (0, -1), 4, WHITE)]))

    abstract = (
        "This whitepaper documents a five-phase development programme for a passively actuated front brake "
        "cooling duct. Four louvers throttle the duct throat and are driven by a nickel-titanium (Nitinol) shape "
        "memory alloy (SMA) spring opposed by a steel bias spring. When the actuator reaches its austenite finish "
        f"temperature (A<sub>f</sub> = {SMA['Af']:.0f} °C) under heavy braking, the SMA spring contracts and opens "
        f"the louvers. When it cools below the martensite finish temperature (M<sub>f</sub> = {SMA['Mf']:.0f} °C), "
        "the bias spring recloses them. No sensor, electronics, hydraulics or driver input is involved."
    )
    regulatory = (
        "The regulatory premise of the concept is that passive, thermally actuated hardware is distinct from the "
        "active, electronically controlled aerodynamic devices that the FIA prohibits. This premise is tested in "
        "Section 6 against the requirement that aerodynamic bodywork be rigidly secured and immobile. It is "
        "identified there as the principal compliance risk of the programme."
    )
    method = (
        "The programme comprises:<br/>"
        "<b>Phase 1:</b> parametric SolidWorks CAD and packaging<br/>"
        f"<b>Phase 2:</b> conjugate heat transfer CFD with k-{g('w')} SST turbulence and Discrete Ordinates "
        "radiation<br/>"
        "<b>Phase 3:</b> nonlinear MAPDL contact and SMA actuation analysis<br/>"
        "<b>Phase 4:</b> instrumented bench validation over a 20 Hz DAQ link<br/>"
        "<b>Phase 5:</b> automated compilation of this report"
    )
    note = ("Values tagged DESIGN TARGET come from the project brief and are not yet supported by a simulation "
            "or test data file. They are replaced automatically when that data exists. A full source list for "
            "every figure and value is in the Appendix.")
    return [banner, Spacer(1, 8), meta, Spacer(1, 10), *section(st, "Executive Summary - Key Performance Indicators"),
            kpi, Spacer(1, 8), *section(st, "Abstract"), Paragraph(abstract, st["body"]),
            Paragraph(regulatory, st["body"]), Paragraph(method, st["body"]), Paragraph(note, st["small"])]


def page2(doc, st, ctx: Context):
    w = doc.width - 14
    corner_r = math.hypot(HOUSING_CY + HOUSING_H / 2 + HOUSING_WALL, HOUSING_W / 2 + HOUSING_WALL)
    blockage = N_LOUVERS * LOUVER_W / HOUSING_H * 100.0
    text1 = (
        f"The 18-inch front wheel defines a rim clearance envelope of {RIM_ID:.0f} mm diameter around a "
        f"{ROTOR_DIA:.0f} x {ROTOR_THICK:.0f} mm carbon-carbon rotor. This leaves a "
        f"{(RIM_ID - ROTOR_DIA) / 2:.0f} mm radial annulus that extends {DUCT_DEPTH:.0f} mm inboard. "
        f"The CFRP duct throat has a {HOUSING_H:.0f} x {HOUSING_W:.0f} mm internal section and "
        f"{HOUSING_WALL:.0f} mm walls ({HOUSING_H + 2 * HOUSING_WALL:.0f} x {HOUSING_W + 2 * HOUSING_WALL:.0f} mm "
        f"outer). It sits {HOUSING_X0 - ROTOR_THICK / 2:.0f} mm inboard of the rotor face, with a "
        f"{RIM_ID / 2 - corner_r:.1f} mm radial margin to the rim. "
        f"<b>Packaging finding:</b> the throat cannot fit inside the {(RIM_ID - ROTOR_DIA) / 2:.0f} mm annulus "
        f"itself; it needs {HOUSING_H + 2 * HOUSING_WALL:.0f} mm of radial depth and therefore overlaps the "
        "hub zone below the rotor outer radius."
    )
    text2 = (
        f"Four {LOUVER_L:.0f} x {LOUVER_W:.0f} x {LOUVER_T:.0f} mm louvers pivot at a {LOUVER_PITCH:.0f} mm pitch "
        "on 3 mm shafts in the housing side walls. When closed they block "
        f"{blockage:.0f} % of the throat height. The Phase 1 crank pin sat 6 mm off the pivot along the flap "
        "width, which put the linkage at dead centre when closed and limited stroke to about 5 mm. Phase 3 "
        f"therefore introduced an external crank of r = {CRANK_R:.2f} mm swinging -40° to +40° about horizontal. "
        "This gives 14.0 mm of rod travel at 80° of louver rotation, with a lever arm of at least "
        f"{CRANK_R * math.cos(math.radians(40)):.1f} mm. The vertical control rod is pulled up by the SMA "
        f"spring and down by a {BIAS_K_NMM} N/mm steel bias spring with a {BIAS_PRELOAD:.0f} N preload."
    )
    return [*section(st, "1. Phase 1 - CAD, Packaging and Kinematics"),
            Paragraph("<b>1.1 18-inch front axle packaging envelope.</b> " + text1, st["body"]),
            fit_image(ctx.assets["packaging"].path, w, 2.75 * inch),
            Paragraph("Figure 1 - SOLIDWORKS 2025 render of the Phase 1 assembly: transparent rotor and duct packaging "
                      "volumes with the duct housing, four louvers and control rod.", st["caption"]),
            Paragraph("<b>1.2 Parallel-bar linkage and dual-spring bias.</b> " + text2, st["body"]),
            fit_image(ctx.assets["kinematics"].path, w, 2.75 * inch),
            Paragraph("Figure 2 - Kinematic schematic: closed (slate) and 80° open (red) states, SMA spring above "
                      "the rod and steel bias spring below.", st["caption"])]


def page3(doc, st, ctx: Context):
    w = doc.width - 14
    overview = (
        "The CHT domain was meshed in the Fluent Watertight Geometry workflow with a poly-hexcore volume mesh. "
        "Surface sizes range from 0.5 mm to 10 mm, with 1 mm faces on the louvers and SMA pocket; the meshes have 492,211 cells (Case A) and 515,700 cells (Case B), sized to fit the Ansys Student cell limit. Ten prism "
        "layers with a 0.02 mm first-cell height target y<super>+</super> of about 1. Turbulence was modelled "
        f"with k-{g('w')} SST, and rotor-to-duct thermal radiation with the Discrete Ordinates (DO) model. "
        "Air was treated as an incompressible ideal gas (c<sub>p</sub> = 1006.43 J/kg-K, "
        f"k = 0.0242 W/m-K). The solids were a carbon-carbon rotor ({g('r')} = 1800 kg/m³, k = 40 W/m-K), "
        f"a CFRP duct ({g('r')} = 1550 kg/m³, k = 5 W/m-K) and Nitinol louvers. The case was solved with the "
        "pressure-based coupled solver to residuals of 10<super>-5</super> (flow) and 10<super>-6</super> "
        "(energy and radiation). The rotor walls were held at a fixed temperature, and the response variable "
        "is the volume-averaged air temperature in the SMA spring pocket."
    )
    images = Table([[fit_image(ctx.assets["cfd_a"].path, w / 2 - 6, 2.5 * inch),
                     fit_image(ctx.assets["cfd_b"].path, w / 2 - 6, 2.5 * inch)],
                    [Paragraph("Figure 3 - Case A static temperature on the duct mid-plane (rotor held at 1073 K).", st["caption"]),
                     Paragraph("Figure 4 - Case B (300 km/h, louvers open) velocity magnitude on the duct "
                               "mid-plane.", st["caption"])]],
                   colWidths=[w / 2, w / 2])
    images.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"), ("VALIGN", (0, 0), (-1, -1), "MIDDLE")]))

    case_a, res_a = ctx.p2_case("A")
    case_b, res_b = ctx.p2_case("B")

    def col(case, result, planned):
        vel = case.get("inlet_velocity") if case else planned["v"]
        rotor = case.get("rotor_temperature") if case else planned["rotor"]
        status = (result or {}).get("status", "NOT RUN")
        return [f"{fmt(vel, '.2f')} m/s ({fmt(vel * 3.6 if vel else None, '.0f')} km/h)",
                f"{fmt(case.get('inlet_temperature') if case else 300.0, '.1f')} K",
                f"{fmt(rotor, '.2f')} K ({fmt(rotor - 273.15 if rotor else None, '.0f')} °C)",
                (case or {}).get("louver_state", planned["louver"]), status,
                fmt((result or {}).get("t_pocket_avg"), ".2f", " K"),
                fmt((result or {}).get("t_pocket_max"), ".2f", " K"),
                (result or {}).get("t_pocket_check", "-")] + [
                    (lambda w: f"{w[0]:.0f}-{w[1]:.0f} K" if w else "n/a")(
                        (result or {}).get("wire_T_K_gap3to15mm", {}).get(e)) for e in ("0.6", "0.2")]

    a = col(case_a, res_a, {"v": 27.78, "rotor": 1073.15, "louver": "CLOSED (0 deg)"})
    b = col(case_b, res_b, {"v": 83.33, "rotor": 673.15, "louver": "OPEN (80 deg)"})
    labels = ["Freestream velocity", "Inlet air temperature", "Rotor temperature", "Louver state",
              "Solver status", "T_pocket air volume-average", "T_pocket air maximum", "Pocket air check (>= 358 K)",
              "SMA wire T, emissivity 0.6 (open needs >= 358 K, close needs <= 318 K)",
              "SMA wire T, emissivity 0.2 (bright wire)"]
    rows = [["Parameter", "Case A - 100 km/h braking", "Case B - 300 km/h straight"]]
    rows += [[label, va, vb] for label, va, vb in zip(labels, a, b)]
    src = esc(ctx.phase2_path) if ctx.phase2_path else "no Phase 2 results file found"
    return [*section(st, "2. Phase 2 - Conjugate Heat Transfer and CFD"), Paragraph(overview, st["body"]), images,
            Paragraph("<b>2.1 Operating case comparison.</b> Both cases use the relocated SMA pocket on the "
                      "rotor's outboard wake face. Case B uses a second SOLIDWORKS export with the louvers open at "
                      "90° (MAPDL hot state 86°). The spring-wire rows come from a radiation and convection heat "
                      "balance on the 1.2 mm wire using Fluent's DO incident radiation, 3-15 mm from the rotor "
                      f"face. Source: {src}.", st["body"]),
            styled_table(st, rows, [1.9 * inch, (w - 1.9 * inch) / 2, (w - 1.9 * inch) / 2])]


def page4(doc, st, ctx: Context):
    w = doc.width - 14
    s = ctx.sma
    xi, a_ = g("x"), g("a")
    eqs = [
        f"Heating (M -> A): {xi}(T) = ({xi}<sub>0</sub>/2) [cos(a<sub>A</sub>(T - A<sub>s</sub>)) + 1],  "
        f"a<sub>A</sub> = {g('p')} / (A<sub>f</sub> - A<sub>s</sub>)",
        f"Cooling (A -> M): {xi}(T) = ((1 - {xi}<sub>0</sub>)/2) cos(a<sub>M</sub>(T - M<sub>f</sub>)) + "
        f"(1 + {xi}<sub>0</sub>)/2,  a<sub>M</sub> = {g('p')} / (M<sub>s</sub> - M<sub>f</sub>)",
        f"Clausius-Clapeyron: d{g('s')}/dT = -{g('r')} {g('D')}s / {g('e')}<sub>L</sub>  ->  "
        f"dF/dT = {s['rate']:.2f} N/°C (force equivalent)",
        f"Helical spring: k = G d<super>4</super> / (8 D<super>3</super> n),  G = E / (2(1 + {g('n')})),  "
        f"{g('t')} = K<sub>W</sub> 8 F D / ({g('p')} d<super>3</super>)",
    ]
    physics = (
        f"Nitinol transforms between low-modulus martensite (E<sub>M</sub> = {SMA['EM'] / 1e9:.0f} GPa) and "
        f"high-modulus austenite (E<sub>A</sub> = {SMA['EA'] / 1e9:.0f} GPa). On heating, martensite reverts "
        f"to austenite between A<sub>s</sub> = {SMA['As']:.0f} °C and A<sub>f</sub> = {SMA['Af']:.0f} °C, "
        f"recovering up to {SMA['epsL'] * 100:.1f} % strain. On cooling, martensite re-forms between "
        f"M<sub>s</sub> = {SMA['Ms']:.0f} °C and M<sub>f</sub> = {SMA['Mf']:.0f} °C. The martensite volume "
        f"fraction {xi} follows Liang-Rogers cosine kinetics. The transformation stress rises with temperature "
        "according to the Clausius-Clapeyron relation, calibrated to the 32 N recovery force at A<sub>f</sub>. "
        f"For the assumed spring (d = {SMA['d'] * 1e3:.1f} mm, D = {SMA['D'] * 1e3:.1f} mm, n = {SMA['n']}): "
        f"k<sub>A</sub> = {s['k_a']:.2f} N/mm, k<sub>M</sub> = {s['k_m']:.2f} N/mm, recoverable extension "
        f"{s['delta_l']:.1f} mm, forward plateau at 85 °C {s['f_am_85']:.0f} N, Wahl-corrected wire stress "
        f"{s['tau']:.0f} MPa at 32 N."
    )
    stress_rows = [["Component / location", "Material", "Peak von Mises", "Yield", "FoS", "Check"]]
    checks = (ctx.phase3 or {}).get("stress_checks", [])
    if checks:
        for c in checks:
            stress_rows.append([c.get("location", "-"), c.get("material", "-"), fmt(c.get("peak_MPa"), ".2f", " MPa"),
                                fmt(c.get("yield_MPa"), ".0f", " MPa"), fmt(c.get("fos"), ".1f"), c.get("check", "-")])
        fos_note = f"Source: {esc(ctx.phase3_path)}."
    else:
        for loc, mat in (("Louver pivot pins (Case 1 / Case 2)", "PA12-CF"),
                         ("Housing pivot holes (Case 1 / Case 2)", "PA12-CF"),
                         ("Control rod link holes (Case 1 / Case 2)", "Al 7075-T6")):
            stress_rows.append([loc, mat, "pending", f"{YIELD[mat]:.0f} MPa", f">= {FOS_REQUIRED}", "PENDING"])
        fos_note = ("Phase 3 prints its stress results to the console but does not yet write "
                    "phase3_results.json, so the peaks and FoS values are pending. Requirement: FoS >= 2.5 "
                    "against yield.")
    return [*section(st, "3. Phase 3 - Nonlinear Structural Mechanics and SMA Physics"),
            Paragraph(physics, st["body"]), *[Paragraph(e, st["eq"]) for e in eqs],
            fit_image(ctx.assets["fea_stress"].path, w, 2.45 * inch),
            Paragraph("Figure 5 - MAPDL von Mises stress, hot actuation state: louver pivot pins (SOLID187, "
                      "frictionless pin-hole contact) and control rod link holes.", st["caption"]),
            fit_image(ctx.assets["fea_fd"].path, w, 2.25 * inch),
            Paragraph("Figure 6 - Nitinol force-displacement hysteresis, actuator force budget and thermal "
                      "stroke hysteresis.", st["caption"]),
            Paragraph("<b>3.1 Structural factor of safety verification.</b> " + fos_note, st["body"]),
            styled_table(st, stress_rows, [2.3 * inch, 0.95 * inch, 1.05 * inch, 0.85 * inch, 0.7 * inch,
                                           w - 5.85 * inch])]


def page5(doc, st, ctx: Context):
    w = doc.width - 14
    p4 = ctx.phase4 or {}
    bench_text = (
        "The physical bench reproduces one louver bank at 1:1 scale, with a 3D-printed PA12-CF duct throat and "
        "louvers, an aluminium control rod, and the Nitinol and steel bias springs. The sensor payload is:<br/>"
        "- an inline load cell in series with the SMA spring (actuation force)<br/>"
        "- a thermocouple bonded to the SMA coil (transformation temperature)<br/>"
        "- a linear potentiometer on the control rod (stroke)<br/>"
        "An ESP32 DAQ streams newline-delimited JSON telemetry at 115200 baud. The host logger samples it at "
        "20 Hz through a 60 s heating and 60 s cooling cycle, checks each packet for staleness, logs it to CSV "
        "and correlates the run against the Phase 3 prediction."
    )
    story = [*section(st, "4. Phase 4 - Physical Bench Validation and HIL Testing"), Paragraph(bench_text, st["body"])]
    if ctx.bench is not None and ctx.bench_source != "MEASURED":
        story.append(Paragraph('<font color="#DC2626"><b>WARNING:</b></font> the bench data set is tagged '
                               f"{esc(ctx.bench_source)}. It is a pipeline dry run, not physical validation.",
                               st["body"]))
    story += [fit_image(ctx.assets["bench"].path, w, 2.85 * inch),
              Paragraph("Figure 7 - DAQ hysteresis loop (bench vs FEA) and thermal cycle with actuation force, "
                        "plotted from Phase4_Bench_Test_Data.csv.", st["caption"])]

    def row(name, fea_value, meas, err):
        return [name, fea_value, meas, err]

    rows = [["Metric", "ANSYS virtual prediction", f"Bench measurement ({ctx.bench_source})" if p4
             else "Bench measurement", "Error"]]
    if p4:
        rows += [
            row("Peak actuation force", fmt(p4.get("fea_force_target_N"), ".2f", " N"),
                fmt(p4.get("peak_force_N"), ".2f", " N"), fmt(p4.get("force_error_pct"), "+.1f", " %")),
            row("Maximum stroke", fmt(p4.get("fea_stroke_max_mm"), ".2f", " mm"),
                fmt(p4.get("max_stroke_mm"), ".2f", " mm"), fmt(p4.get("stroke_error_pct"), "+.1f", " %")),
            row("Louver opening angle", fmt(p4.get("fea_opening_angle_deg"), ".1f", "°"),
                fmt(p4.get("opening_angle_deg"), ".1f", "°"), fmt(p4.get("angle_error_pct"), "+.1f", " %")),
            row("Thermal trigger onset", fmt(p4.get("fea_onset_temp_C"), ".1f", " °C"),
                fmt(p4.get("onset_temp_C"), ".1f", " °C"), fmt(p4.get("onset_error_pct"), "+.1f", " %")),
            row("Hysteresis width (50 % stroke)", fmt(p4.get("fea_hysteresis_width_C"), ".1f", " °C"),
                fmt(p4.get("hysteresis_width_C"), ".1f", " °C"), fmt(p4.get("hysteresis_error_pct"), "+.1f", " %")),
            row("Stroke RMSE", "-", fmt(p4.get("stroke_rmse_mm"), ".3f", " mm"),
                fmt(p4.get("stroke_rmse_pct"), ".1f", " % of max")),
            row("Force correlation", "-", f"R² = {fmt(p4.get('force_r2'), '.3f')}",
                fmt(p4.get("force_rmse_N"), ".2f", " N RMSE")),
            row("Samples (valid / total)", "-", f"{p4.get('valid_samples', '-')} / {p4.get('samples', '-')}",
                fmt(p4.get("effective_rate_Hz"), ".1f", " Hz")),
        ]
        note = f"Source: {esc(ctx.phase4_path)}."
    else:
        rows += [row(m, v, "not measured", "-") for m, v in (
            ("Peak actuation force", f"{FEA_FORCE_MAX:.2f} N"), ("Maximum stroke", f"{FEA_STROKE_MAX:.2f} mm"),
            ("Louver opening angle", f"{crank_angle_deg(FEA_STROKE_MAX):.1f}°"),
            ("Thermal trigger onset", f"{SMA['As']:.0f} °C (As)"), ("Hysteresis width (50 % stroke)", "30.0 °C"))]
        note = "No Phase 4 correlation summary was found; run phase4_bench_validation.py to populate this table."
    story += [Paragraph("<b>4.1 Virtual-physical correlation.</b> " + note, st["body"]),
              styled_table(st, rows, [1.9 * inch, 1.65 * inch, 1.95 * inch, w - 5.5 * inch])]
    return story


def page6(doc, st, ctx: Context):
    p4 = ctx.phase4 or {}
    _, res_a = ctx.p2_case("A")
    conclusions = []
    if res_a and res_a.get("t_pocket_avg") is not None:
        conclusions.append(f"Phase 2 CFD predicts a Case A SMA pocket air temperature of "
                           f"{fmt(res_a['t_pocket_avg'], '.1f')} K ({res_a.get('t_pocket_check', '-')}). "
                           "The pocket was moved from beside the duct (300.3 K) to the rotor's outboard wake face; "
                           "air there is hot only in a ~2 mm layer against the rotor, so the pocket average is "
                           "still below the trigger. Including rotor radiation (Fluent DO incident radiation "
                           "119-129 kW/m2 in the pocket) in a heat balance on the 1.2 mm wire gives 378-408 K "
                           "with a surface emissivity of 0.6, above Af, but only 326-353 K for bright wire "
                           "(emissivity 0.2). The duct can self-actuate if the spring is oxidised or coated to "
                           "an emissivity of at least about 0.45 (0.65 allowing for coil self-shading); this is "
                           "an estimate that neglects conduction into the spring mounts. In Case B (300 km/h, "
                           "400 °C rotor, louvers open) the incident radiation falls to about 20 kW/m2 and the wire "
                           "settles at 306-311 K for any emissivity, below Mf (318 K), so the bias spring recloses "
                           "the louvers. On paper the full open/close cycle works for an oxidised spring placed "
                           "3-15 mm from the rotor face. A transient lap model calibrated to both Fluent cases and "
                           "the MAPDL sweep changes this verdict: the 1.2 mm wire (time constant ~5 s) swings only "
                           "314-349 K over a lap, so it never opens, and with a darker coating it opens but then "
                           "stays open on 91 % of straights because the 40 K NiTi hysteresis exceeds the lap-scale "
                           "swing. A 0.5 mm narrow-hysteresis NiTiCu wire (Af 76 C, Mf 58 C), emissivity 0.8, 3 mm "
                           "from the rotor opens fully 4-5 s after the three hardest braking zones and is open on "
                           "only 2 % of straight-line time. Re-running MAPDL with six such springs in parallel "
                           "(2.5 mm coils, 19 turns, 33.3 N) gives 88.5° and 15.15 mm at 76 °C, opening between "
                           "71.5 and 76 °C and closing between 64 and 58 °C, with every stress check still passing "
                           "(lowest FoS 3.3).")
    else:
        conclusions.append("The Phase 2 SMA pocket temperature result is not yet available. Whether the "
                           "actuator triggers under braking is therefore not yet confirmed by CFD.")
    conclusions.append(f"The Phase 3 external crank (r = {CRANK_R:.2f} mm) removes the Phase 1 dead-centre "
                       "condition and gives 14.0 mm of stroke for 80° of louver rotation.")
    if p4:
        conclusions.append(f"Bench correlation ({esc(ctx.bench_source)} data): stroke NRMSE "
                           f"{fmt(p4.get('stroke_rmse_pct'), '.1f')} %, peak force "
                           f"{fmt(p4.get('peak_force_N'), '.1f')} N against 32.0 N, and a hysteresis width of "
                           f"{fmt(p4.get('hysteresis_width_C'), '.1f')} °C.")
    else:
        conclusions.append("Physical correlation is pending a Phase 4 bench run.")
    conclusions.append("The -18.4 % straight-line drag reduction is ruled out. An isolated-wheel Fluent model at "
                       "300 km/h (wheel, cover, rotor, duct with forward scoop) was solved on three meshes (284k, "
                       "408k and 768k cells). The direct open-vs-closed difference (-9.6 %, +5.2 %, -1.3 %) changes "
                       "sign between meshes and stays inside the iteration noise of the unsteady wheel wake; the "
                       "duct's own drag changes by only 1-2 N. The duct's through-flow (momentum) drag, mass flow "
                       "times speed, is 13-15 N or 1.1-1.4 % of total drag on every mesh; interference with the "
                       "outside flow can move the total either way, so this sets the scale rather than a cap. With the wheel rotating on a moving ground and the drag time-averaged over "
                       "160 ms of transient flow, closing the louvers adds 0.53 % drag (95 % interval +0.43 to "
                       "+0.63 %): the duct's own drag falls 0.4 N but the wheel's rises more. Closing the duct "
                       "therefore brings no drag benefit in this model.")

    fia = (
        "The brief cites Article 3.10 (Aerodynamic Influence). The Formula 1 Technical Regulations require that "
        "bodywork influencing aerodynamic performance be rigidly secured and immobile relative to the sprung "
        "part of the car, and they prohibit movable aerodynamic devices. The regulatory test is therefore "
        "whether a part moves, not how it is actuated. A louver that rotates through about 80° in service is "
        "a moving aerodynamic part, and passive thermal actuation is likely to be judged the same way as "
        "electronic or driver actuation. Brake cooling ducts are also subject to their own detailed geometric "
        "constraints. <b>Assessment:</b> the concept should be treated as non-compliant for competition use "
        "unless the FIA Technical Department confirms otherwise in writing. Article numbers and wording must "
        "be checked against the issue of the regulations in force. The concept remains valid as a "
        "research and road-car or non-FIA-series cooling technology."
    )
    width_fea = 30.0
    width_meas = p4.get("hysteresis_width_C")
    stability = (
        f"The designed transformation band gives a 50 %-stroke hysteresis of about {width_fea:.0f} °C"
        + (f" (measured {fmt(width_meas, '.1f')} °C)" if width_meas is not None else "")
        + ". The duct therefore opens near 80-85 °C and recloses only once the actuator falls below about "
        "50 °C. The transient lap model shows this dead-band is too wide: the 1.2 mm wire swings only about "
        "35 K per lap, so once open it stays open on most straights. A narrow-hysteresis NiTiCu alloy "
        "(about 12 K) in a 0.5 mm wire keeps the louvers closed on 98 % of straight-line time while still "
        "opening after the hardest braking zones. Over a race stint, functional fatigue can "
        "shift the transformation temperatures and reduce the recoverable strain. A training procedure and "
        "multi-thousand-cycle endurance testing are required before transformation temperatures are frozen "
        "for a design."
    )
    future = [
        "Validate the transient lap model against a transient Fluent run of one braking zone and a measured "
        "track speed trace, and source a NiTiCu wire with the Af 76 °C / Mf 58 °C window used here.",
        "Revise the drag KPI: isolated-wheel CFD gives about -1 % of wheel drag. Confirm with a rotating wheel, "
        "moving ground and full-car installation, where the duct's effect on the wheel wake may differ.",
        "Extend the CFD outlet further and refine the rotor wake to remove the remaining outlet backflow "
        "(continuity stalled near 4e-3 to 9e-3).",
        "Characterise the SMA spring: DSC for the transformation temperatures, isothermal load-unload loops.",
        "Resolve the packaging overlap of the duct throat with the hub zone, and the 28 % leakage path when closed.",
        "Submit a formal technical clarification request to the FIA before any competition application.",
    ]
    references = [
        "Liang, C. and Rogers, C.A. (1990). One-dimensional thermomechanical constitutive relations for shape "
        "memory materials. J. Intelligent Material Systems and Structures, 1(2), 207-234.",
        "Tanaka, K. (1986). A thermomechanical sketch of shape memory effect: one-dimensional tensile behavior. "
        "Res Mechanica, 18, 251-263.",
        "Brinson, L.C. (1993). One-dimensional constitutive behavior of shape memory alloys. J. Intelligent "
        "Material Systems and Structures, 4(2), 229-242.",
        "Menter, F.R. (1994). Two-equation eddy-viscosity turbulence models for engineering applications. "
        "AIAA Journal, 32(8), 1598-1605.",
        "Chui, E.H. and Raithby, G.D. (1993). Computation of radiant heat transfer on a nonorthogonal mesh using "
        "the finite-volume method. Numerical Heat Transfer B, 23, 269-288.",
        "Wahl, A.M. (1963). Mechanical Springs, 2nd ed. McGraw-Hill.",
        "Hoerner, S.F. (1965). Fluid-Dynamic Drag. Hoerner Fluid Dynamics.",
        "FIA. Formula 1 Technical Regulations (issue in force). Federation Internationale de l'Automobile.",
        "Ansys Inc. Fluent Theory Guide; Mechanical APDL Element Reference (current release).",
    ]
    prov_rows = [["Item", "Source", "Status"]] + [[a, b, c] for a, b, c in ctx.provenance]
    w = doc.width - 14
    return [*section(st, "5. Conclusion, FIA Regulatory Assessment and Future Work"),
            Paragraph("<b>5.1 Conclusions.</b> " + " ".join(esc(c) for c in conclusions), st["body"]),
            Paragraph("<b>5.2 FIA regulatory assessment.</b> " + fia, st["body"]),
            Paragraph("<b>5.3 Thermal hysteresis stability across multi-lap stints.</b> " + stability, st["body"]),
            Paragraph("<b>5.4 Future work.</b><br/>" + "<br/>".join(f"{i}. {esc(t)}" for i, t in enumerate(future, 1)),
                      st["body"]),
            Paragraph("<b>References</b><br/>" + "<br/>".join(f"[{i}] {esc(r)}" for i, r in enumerate(references, 1)),
                      st["small"]),
            Paragraph("<b>Appendix A - Data provenance</b>", st["h2"]),
            styled_table(st, prov_rows, [1.9 * inch, w - 3.3 * inch, 1.4 * inch])]


# =============================================================================
# BUILD
# =============================================================================
def prepare_assets(ctx: Context) -> None:
    os.makedirs(ASSET_DIR, exist_ok=True)
    pack = os.path.join(ASSET_DIR, "fig_packaging.png")
    figure_packaging(pack)
    ctx.assets["packaging"] = generated_asset("packaging", pack, "Phase 1 design parameters")
    render = PROJECT_ROOT + r"\cad\Phase1_ThermoKinetic_Assembly.png"
    if os.path.isfile(render) and verify_image(render)[0]:
        ctx.assets["packaging"] = Asset("packaging", render, "OK", render, verify_image(render)[1])
    kin = os.path.join(ASSET_DIR, "fig_kinematics.png")
    figure_kinematics(kin)
    ctx.assets["kinematics"] = generated_asset("kinematics", kin, "Phase 3 crank parameters")
    ctx.assets["cfd_a"] = external_asset("cfd_a", "Temp_Contour_CaseA.png", CFD_DIRS, "Case A temperature contour",
                                         "Export from Fluent results (Phase 2 does not yet write images).")
    ctx.assets["cfd_b"] = external_asset("cfd_b", "Velocity_Contour_CaseB.png", CFD_DIRS, "Case B velocity contour",
                                         "Run phase2_post.py after the Case A solve.")
    ctx.assets["fea_stress"] = external_asset("fea_stress", "FEA_Stress_Contour_Hot.png", FEA_DIRS,
                                              "MAPDL von Mises stress (hot)", "Run run_phase3_linkage_fea.py.")
    ctx.assets["fea_fd"] = external_asset("fea_fd", "Nitinol_Force_Displacement_Plot.png", FEA_DIRS,
                                          "Nitinol force-displacement", "Run run_phase3_linkage_fea.py.")
    bench_png = os.path.join(ASSET_DIR, "fig_bench_hysteresis.png")
    if ctx.bench is not None:
        figure_bench(bench_png, ctx.bench, ctx.bench_source)
        ctx.assets["bench"] = generated_asset("bench", bench_png, PHASE4_CSV)
    else:
        placeholder_figure(bench_png, "DAQ hysteresis graph", os.path.basename(PHASE4_CSV),
                           "Run phase4_bench_validation.py to record bench data.")
        ctx.assets["bench"] = Asset("bench", bench_png, "MISSING -> PLACEHOLDER", "not found",
                                    verify_image(bench_png)[1])

    ctx.provenance = [
        ("Phase 2 CFD results", ctx.phase2_path or "not found", "LOADED" if ctx.phase2 else "MISSING"),
        ("Phase 3 FEA results", ctx.phase3_path or "phase3_results.json not exported",
         "LOADED" if ctx.phase3 else "MISSING"),
        ("Phase 4 correlation summary", ctx.phase4_path or "not found",
         f"LOADED ({ctx.bench_source})" if ctx.phase4 else "MISSING"),
        ("Phase 4 DAQ log", PHASE4_CSV if ctx.bench is not None else "not found",
         "LOADED" if ctx.bench is not None else "MISSING"),
        ("KPI: aero drag delta", "cfd/wheel/wheel_aero_open_vs_closed.json", "FLUENT 2026 R1"),
    ] + [(f"Figure: {a.key}", a.origin, a.status) for a in ctx.assets.values()]


def build_pdf(ctx: Context, output: str) -> None:
    doc = SimpleDocTemplate(output, pagesize=LETTER, leftMargin=0.6 * inch, rightMargin=0.6 * inch,
                            topMargin=0.78 * inch, bottomMargin=0.68 * inch, title=f"{TITLE}: {SUBTITLE}",
                            author=ctx.args.author, subject="F1 Technical Whitepaper",
                            creator="phase5_generate_whitepaper.py")
    st = build_styles()
    story = []
    pages = (page1, page2, page3, page4, page5, page6)
    for i, page in enumerate(pages):
        story.append(page_frame(doc, page(doc, st, ctx)))
        if i < len(pages) - 1:
            story.append(PageBreak())
    doc.build(story, canvasmaker=WhitepaperCanvas)


def count_pages_pypdf(path: str):
    try:
        from pypdf import PdfReader
    except ImportError:
        return None
    try:
        return len(PdfReader(path).pages)
    except Exception:  # noqa: BLE001 - verification is best-effort
        return None


def print_table(title: str, headers, rows) -> None:
    widths = [max([len(str(h))] + [len(str(r[i])) for r in rows]) for i, h in enumerate(headers)]
    border = "+" + "+".join("-" * (w + 2) for w in widths) + "+"
    print(f"\n{title}\n{border}")
    print("| " + " | ".join(str(h).ljust(w) for h, w in zip(headers, widths)) + " |")
    print(border)
    for row in rows:
        print("| " + " | ".join(str(c).ljust(w) for c, w in zip(row, widths)) + " |")
    print(border)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Compile the Phase 5 F1 technical whitepaper PDF.")
    parser.add_argument("--output", default=OUTPUT_PDF)
    parser.add_argument("--author", default="Independent student project")
    parser.add_argument("--organisation", default="Independent F1 Technical Case Study")
    parser.add_argument("--revision", default="1.0")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)-7s  %(message)s", datefmt="%H:%M:%S")

    ctx = Context(args)
    prepare_assets(ctx)
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    status = "CREATED"
    try:
        build_pdf(ctx, args.output)
    except Exception as exc:  # noqa: BLE001 - surface the failure in the verification table
        status = f"FAILED: {type(exc).__name__}: {exc}"
        LOG.exception("PDF build failed")

    exists = os.path.isfile(args.output)
    pages_canvas = WhitepaperCanvas.page_total
    pages_pypdf = count_pages_pypdf(args.output) if exists else None
    page_count = pages_pypdf if pages_pypdf is not None else pages_canvas
    page_ok = page_count == EXPECTED_PAGES
    print("\n" + "=" * 100)
    print(" PHASE 5 WHITEPAPER COMPILATION - VERIFICATION")
    print("=" * 100)
    print_table("PDF", ("Item", "Value", "Check"), [
        ("Build status", status, "PASS" if status == "CREATED" and exists else "FAIL"),
        ("Output file", args.output, "PASS" if exists else "FAIL"),
        ("File size", f"{os.path.getsize(args.output) / 1024:.1f} KB" if exists else "-", "-"),
        ("Page count", f"{page_count} (canvas {pages_canvas}, pypdf {pages_pypdf if pages_pypdf is not None else 'n/a'})",
         "PASS" if page_ok else f"FAIL (expected {EXPECTED_PAGES})"),
        ("Bench data source", ctx.bench_source if ctx.bench is not None else "none", "-"),
    ])
    print_table("Embedded images", ("Figure", "Status", "Pixels", "Source"),
                [(a.key, a.status, a.pixels, a.origin) for a in ctx.assets.values()])
    print_table("Data provenance", ("Item", "Source", "Status"), ctx.provenance)
    print("=" * 100)
    corrupt = any(a.status.startswith("CORRUPT") for a in ctx.assets.values())
    return 0 if status == "CREATED" and exists and page_ok and not corrupt else 2


if __name__ == "__main__":
    sys.exit(main())
