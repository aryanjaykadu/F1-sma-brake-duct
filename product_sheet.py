"""Phase 7d - one-page product sheet (A4 PDF) for the SMA brake-duct actuator module and its bench rig."""
from paths import ROOT as PROJECT_ROOT
import json
import os

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.platypus import Image, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle

ROOT = PROJECT_ROOT
P = lambda *a: os.path.join(ROOT, *a)  # noqa: E731
OUT = P("report", "SMA_Duct_Module_Product_Sheet.pdf")

camp = json.load(open(P("bench", "virtual", "virtual_campaign_results.json")))
fea = json.load(open(P("fea", "pack_NiTiCu", "phase3_results.json")))
t1, t2, t3 = camp["T1_hysteresis"], camp["T2_step"], camp["T3_lap"]

ink = colors.HexColor("#1d1d1f")
accent = colors.HexColor("#d9480f")
grey = colors.HexColor("#6b6b70")
h1 = ParagraphStyle("h1", fontName="Helvetica-Bold", fontSize=17, leading=20, textColor=ink)
sub = ParagraphStyle("sub", fontName="Helvetica", fontSize=9.5, leading=12, textColor=grey)
h2 = ParagraphStyle("h2", fontName="Helvetica-Bold", fontSize=10.5, leading=13, textColor=accent, spaceBefore=4)
body = ParagraphStyle("body", fontName="Helvetica", fontSize=8.4, leading=10.6, textColor=ink)
cap = ParagraphStyle("cap", fontName="Helvetica", fontSize=7.2, leading=9, textColor=grey, alignment=1)


def img(path, w):
    from PIL import Image as PI
    im = PI.open(path)
    # content = clearly darker than the light SOLIDWORKS gradient background
    x0, y0, x1, y1 = im.convert("L").point(lambda v: 255 if v < 200 else 0).getbbox() or (0, 0, *im.size)
    pad = 12
    crop = im.crop((max(0, x0 - pad), max(0, y0 - pad), min(im.width, x1 + pad), min(im.height, y1 + pad)))
    tmp = path.replace(".png", "_crop.png")
    crop.save(tmp)
    return Image(tmp, width=w, height=w * crop.height / crop.width)


def table(rows, widths):
    key = ParagraphStyle("key", parent=body, fontName="Helvetica-Bold")
    rows = [[Paragraph(a, key), Paragraph(b, body)] for a, b in rows]
    t = Table(rows, colWidths=widths)
    t.setStyle(TableStyle([("FONT", (0, 0), (-1, -1), "Helvetica", 7.8), ("FONT", (0, 0), (0, -1), "Helvetica-Bold", 7.8),
                           ("TEXTCOLOR", (0, 0), (-1, -1), ink), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("LINEBELOW", (0, 0), (-1, -2), 0.25, colors.HexColor("#d0d0d4")),
                           ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
    return t


act = fea["actuation"]
spec = [
    ["Function", "Opens brake-cooling louvers when the brakes are hot, closes them on straights. No electronics, no power."],
    ["Actuator", "6 x NiTiCu springs, 0.5 mm wire, 2.5 mm coil, 19 turns, black high-emissivity coating, steel bias spring"],
    ["Trigger", f"Opens {t1['heating_5pct_C']:.0f}-{t1['heating_95pct_C']:.0f} °C, closes {t1['cooling_95pct_C']:.0f}-"
                f"{t1['cooling_5pct_C']:.0f} °C (hysteresis {t1['hysteresis_width_50pct_C']:.1f} °C)"],
    ["Output", f"{act['hot_stroke_mm']:.2f} mm rod stroke, louvers 0 to {act['hot_theta_deg']:.1f}°, "
               f"{act['hot_sma_force_N']:.1f} N pack force (MAPDL)"],
    ["Response", f"Opens in {t2['wire_0.5mm']['open_90pct_s']:.1f} s, closes in {t2['wire_0.5mm']['close_10pct_s']:.1f} s "
                 f"(1.2 mm wire: {t2['wire_1.2mm']['close_10pct_s']:.1f} s to close)"],
    ["Structure", "All stress checks pass, lowest factor of safety 3.3 (ANSYS MAPDL)"],
    ["Envelope", "89 x 182 x 94 mm module: CFRP throat, 4 titanium louvers, crank linkage, Al inlet flange (4 x M3)"],
]
results = [
    ["Thermal CFD", "Rotor radiation at the spring pocket 119-129 kW/m² (Fluent DO); air alone cannot trigger the spring"],
    ["Lap model", "Opens 4-5 s after the 3 hardest braking zones, open 2 % of straight-line time"],
    ["Aero CFD", "Rotating wheel on a moving road at 300 km/h, time-averaged: closing the louvers adds 0.5 % drag; "
                 "the duct's own momentum drag is only 1.1-1.4 % of the total (brief claimed 18.4 %)"],
    ["Bench rig", "Lamp + blower rig, Arduino shield (ERC/DRC clean, Gerbers), firmware 57 % flash / 55 % RAM"],
    ["Digital twin", f"Same firmware code + physics: reproduces the car lap's openings ({t3['rig_vs_car_open_state_agreement_pct']:.0f} % "
                     "agreement); 1,000 cycles, 0 faults; caught 3 design errors before build"],
]

story = [Paragraph("SMA Brake-Duct Actuator Module", h1),
         Paragraph("Passive, temperature-driven louver actuator for brake cooling: design, simulation and test-rig "
                   "package. Prototype design, fabrication-ready; not yet built.", sub), Spacer(1, 4 * mm)]
hero = Table([[img(P("cad", "product", "Module_trimetric.png"), 50 * mm),
               img(P("cad", "product", "anim", "frame_00.png"), 47 * mm),
               img(P("cad", "product", "anim", f"frame_{13:02d}.png"), 47 * mm)],
              [Paragraph("Module (SOLIDWORKS)", cap), Paragraph("Cold: louvers closed", cap),
               Paragraph("Hot: louvers open 88.5°", cap)]], colWidths=[64 * mm, 60 * mm, 60 * mm])
story += [hero, Spacer(1, 2 * mm), Paragraph("Specification", h2), table(spec, [24 * mm, 160 * mm]),
          Paragraph("Validation", h2), table(results, [24 * mm, 160 * mm]), Spacer(1, 3 * mm)]
bottom = Table([[img(P("cad", "rig", "Bench_Rig_iso.png"), 44 * mm),
                 img(P("electronics", "kicad", "render_iso.png"), 56 * mm),
                 img(P("bench", "virtual", "T1_hysteresis.png"), 66 * mm)],
                [Paragraph("Bench rig (SOLIDWORKS)", cap), Paragraph("Sensor shield (KiCad 3D)", cap),
                 Paragraph("Digital-twin hysteresis test", cap)]], colWidths=[60 * mm, 60 * mm, 66 * mm])
story += [bottom, Spacer(1, 3 * mm),
          Paragraph("Tools: SOLIDWORKS 2025, ANSYS Fluent and MAPDL (Student 2026 R1), KiCad 10, Arduino (arduino-cli), "
                    "C++/MSVC, Python. Drawing SMA-DM-001, CAD, Gerbers, firmware and test scripts in the project "
                    "repository. Rig and twin results are predictions pending a physical build.", sub)]

SimpleDocTemplate(OUT, pagesize=A4, leftMargin=13 * mm, rightMargin=13 * mm, topMargin=12 * mm, bottomMargin=10 * mm,
                  title="SMA Brake-Duct Actuator Module").build(story)
print("WROTE", OUT)
