"""Phase 1 - build the F1 thermo-kinetic brake duct CAD in the running SOLIDWORKS session.

Parts (wheel frame: X = axle/inboard, Y = up, Z = fore-aft; SI units):
  F1_Brake_Packaging_Zone  - rotor keep-out + 150 mm duct annulus, 75 % transparent
  Brake_Duct_Housing       - 100 x 60 mm throat, 2 mm CFRP wall (shelled outward, ends open),
                             4 pivot hole pairs @ 20 mm, SMA + bias spring anchor pegs
  Nitinol_Louver_Flap      - 56 x 18 x 2 mm flap, 3 mm pivot shafts, external crank arm
                             (r = 10.89 mm, -40 deg when closed) with 2.5 mm crank pin
  Control_Linkage_Rod      - 80 x 6 x 2 mm bar, 4 x 2.6 mm holes @ 20 mm, SMA and bias pegs
Exports STEP (for FEA) and PNG renders of every part.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import math
import os
import sys

import pythoncom
from win32com.client import VARIANT

from swlib import END_MID_PLANE, START_OFFSET, SW, call, dbl_array, get, nothing

OUT = PROJECT_ROOT + r"\cad"

# ---- design parameters (m) ----
ROTOR_D, ROTOR_T, RIM_D, DUCT_DEPTH = 0.328, 0.032, 0.400, 0.150
H_H, H_W, H_LEN, H_X0, H_CY, WALL = 0.100, 0.060, 0.080, 0.018, 0.140, 0.002
PITCH, N_LOUVER = 0.020, 4
HOLE_R = 0.0015
L_L, L_W, L_T = 0.056, 0.018, 0.002
SHAFT_R, SHAFT_LEN = 0.0015, 0.005
CRANK_R = 0.014 / (2.0 * math.sin(math.radians(40.0)))
CRANK_PHI0 = -math.radians(40.0)
CRANK_W, CRANK_T = 0.005, 0.002
CPIN_R, CPIN_LEN = 0.00125, 0.004
ROD_L, ROD_W, ROD_T = 0.080, 0.006, 0.002
ROD_HOLE_R = 0.0013
ROD_PEG_R, ROD_PEG_LEN = 0.0008, 0.003
PEG_R, PEG_LEN = 0.0025, 0.010

HX = H_X0 + H_LEN / 2.0
OFFSETS = [(k - (N_LOUVER - 1) / 2.0) * PITCH for k in range(N_LOUVER)]
ROD_X = HX + CRANK_R * math.cos(CRANK_PHI0)          # rod centreline at closed (asm X)


def render(part, path):
    part.view_iso()
    part.doc.GraphicsRedraw2()
    part.export(path)


def build_zone(sw):
    p = sw.new_part()
    sk = p.sketch_on(p.plane(3))
    p.circles(sk, [(0, 0, 0)], ROTOR_D / 2)
    p.extrude(p.close_sketch(), ROTOR_T, end=END_MID_PLANE, merge=False, name="Rotor_Volume_Extrude")
    sk = p.sketch_on(p.plane(3))
    p.circles(sk, [(0, 0, 0)], ROTOR_D / 2)
    p.circles(sk, [(0, 0, 0)], RIM_D / 2)
    p.extrude(p.close_sketch(), DUCT_DEPTH, merge=False, name="Duct_Packaging_Extrude")
    p.rebuild()
    bodies = p.bodies()
    props = [128 / 255.0] * 3 + [1.0, 1.0, 0.5, 0.3125, 0.75, 0.0]
    p.doc.MaterialPropertyValues = dbl_array(props)
    names = []
    for b in bodies:
        box = get(b, "GetBodyBox")
        nm = "Available_Duct_Packaging" if abs(box[3] - box[0]) > 1.5 * ROTOR_T else "Brake_Rotor_Volume"
        b.Name = nm
        names.append(nm)
        try:
            call(b, "SetMaterialPropertyValues2", dbl_array(props), 2, VARIANT(pythoncom.VT_EMPTY, None))
        except Exception as exc:  # noqa: BLE001
            print("   body appearance fallback (part-level):", exc)
    p.rebuild()
    path = os.path.join(OUT, "F1_Brake_Packaging_Zone.SLDPRT")
    p.save(path)
    render(p, os.path.join(OUT, "F1_Brake_Packaging_Zone.png"))
    return {"file": path, "bodies": names}


def build_housing(sw):
    p = sw.new_part()
    sk = p.sketch_on(p.plane(3))
    p.center_rect(sk, (0, H_CY, 0), (0, H_CY + H_H / 2, H_W / 2))
    p.extrude(p.close_sketch(), H_LEN, start=START_OFFSET, start_offset=H_X0, name="Duct_Throat")
    box = get(p.bodies()[0], "GetBodyBox")
    x0, x1 = min(box[0], box[3]), max(box[0], box[3])
    if abs(x0 - H_X0) > 1e-6 or abs(x1 - (H_X0 + H_LEN)) > 1e-6:
        raise RuntimeError(f"throat placed at x = {x0:.4f}..{x1:.4f}")
    p.doc.ClearSelection2(True)
    for i, x in enumerate((x0, x1)):
        p.select_face_at(x, H_CY + H_H / 4, H_W / 4, append=i > 0)
    p.doc.InsertFeatureShell(WALL, True)
    shell = p.last_feature("Shell")
    if shell is None:
        raise RuntimeError("shell failed")
    shell.Name = "Carbon_Wall_Shell"
    p.rebuild()
    sbox = get(p.bodies()[0], "GetBodyBox")
    print("   shelled body box:", [round(v, 5) for v in sbox])
    if sbox[4] < H_CY + H_H / 2 + WALL - 1e-6:
        raise RuntimeError("shell did not grow outward")
    sk = p.sketch_on(p.plane(1))
    p.circles(sk, [(HX, H_CY + o, 0) for o in OFFSETS], HOLE_R)
    p.cut_through_all(p.close_sketch(), name="Pivot_Hole_Pairs")
    y_top, y_bot = H_CY + H_H / 2 + WALL, H_CY - H_H / 2 - WALL
    z_peg = H_W / 2 + WALL - PEG_R - 0.0005
    p.boss_on_face((x0 + 0.01, y_top, 0.0), (ROD_X, y_top, z_peg), PEG_R, PEG_LEN, "SMA_Anchor_Peg")
    p.boss_on_face((x0 + 0.01, y_bot, 0.0), (ROD_X, y_bot, z_peg), PEG_R, PEG_LEN, "Bias_Anchor_Peg")
    p.rebuild()
    path = os.path.join(OUT, "Brake_Duct_Housing.SLDPRT")
    p.save(path)
    p.export(os.path.join(OUT, "Brake_Duct_Housing.STEP"))
    render(p, os.path.join(OUT, "Brake_Duct_Housing.png"))
    return {"file": path, "throat_x": [x0, x1], "pivot_x": HX}


def build_louver(sw):
    p = sw.new_part()
    sk = p.sketch_on(p.plane(1))
    p.center_rect(sk, (0, 0, 0), (L_L / 2, L_W / 2, 0))
    p.extrude(p.close_sketch(), L_T, end=END_MID_PLANE, name="Louver_Flap")
    xe = L_L / 2
    p.boss_on_face((-xe, L_W / 4, 0), (-xe, 0, 0), SHAFT_R, SHAFT_LEN, "Pivot_Shaft_L")
    p.boss_on_face((xe, L_W / 4, 0), (xe, 0, 0), SHAFT_R, SHAFT_LEN, "Pivot_Shaft_R")
    # external crank arm on the +X shaft end; part frame: asm X = -part Z, asm Y = part Y
    xs = xe + SHAFT_LEN
    u = (0.0, math.sin(CRANK_PHI0), -math.cos(CRANK_PHI0))
    w = (0.0, -u[2], u[1])
    sk = p.sketch_on_face(xs, 0.0004, 0.0004)
    pts = []
    for a, b in ((-CRANK_W / 2, -CRANK_W / 2), (CRANK_R + CRANK_W / 2, -CRANK_W / 2),
                 (CRANK_R + CRANK_W / 2, CRANK_W / 2), (-CRANK_W / 2, CRANK_W / 2)):
        pts.append(p.to_sketch(sk, xs, a * u[1] + b * w[1], a * u[2] + b * w[2]))
    p.sm.AddToDB = True
    for i in range(4):
        (ax, ay), (bx, by) = pts[i], pts[(i + 1) % 4]
        if p.sm.CreateLine(ax, ay, 0.0, bx, by, 0.0) is None:
            raise RuntimeError("crank outline failed")
    p.sm.AddToDB = False
    p.extrude(p.close_sketch(), CRANK_T, name="Crank_Arm")
    xc = xs + CRANK_T
    pin = (xc, CRANK_R * u[1], CRANK_R * u[2])
    p.boss_on_face((xc, 0.5 * CRANK_R * u[1], 0.5 * CRANK_R * u[2]), pin, CPIN_R, CPIN_LEN, "Crank_Pin")
    p.rebuild()
    nb = len(p.bodies())
    if nb != 1:
        raise RuntimeError(f"louver has {nb} bodies")
    path = os.path.join(OUT, "Nitinol_Louver_Flap.SLDPRT")
    p.save(path)
    p.export(os.path.join(OUT, "Nitinol_Louver_Flap.STEP"))
    render(p, os.path.join(OUT, "Nitinol_Louver_Flap.png"))
    return {"file": path, "crank_pin_part": pin, "crank_face_x": xc}


def build_rod(sw):
    p = sw.new_part()
    sk = p.sketch_on(p.plane(1))
    p.center_rect(sk, (0, 0, 0), (ROD_W / 2, ROD_L / 2, 0))
    p.extrude(p.close_sketch(), ROD_T, name="Rod_Bar")
    sk = p.sketch_on(p.plane(1))
    p.circles(sk, [(0, o, 0) for o in OFFSETS], ROD_HOLE_R)
    p.cut_through_all(p.close_sketch(), name="Link_Holes")
    p.boss_on_face((ROD_W / 4, ROD_L / 2, ROD_T / 2), (0, ROD_L / 2, ROD_T / 2), ROD_PEG_R, ROD_PEG_LEN,
                   "SMA_Spring_Peg")
    p.boss_on_face((ROD_W / 4, -ROD_L / 2, ROD_T / 2), (0, -ROD_L / 2, ROD_T / 2), ROD_PEG_R, ROD_PEG_LEN,
                   "Bias_Spring_Peg")
    p.rebuild()
    path = os.path.join(OUT, "Control_Linkage_Rod.SLDPRT")
    p.save(path)
    p.export(os.path.join(OUT, "Control_Linkage_Rod.STEP"))
    render(p, os.path.join(OUT, "Control_Linkage_Rod.png"))
    return {"file": path}


def main(which):
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    builders = {"zone": build_zone, "housing": build_housing, "louver": build_louver, "rod": build_rod}
    report = {}
    for key in which:
        for f in os.listdir(OUT):
            pass
        print(f"== building {key}")
        report[key] = builders[key](sw)
        print("   ok:", report[key])
    with open(os.path.join(OUT, "phase1_parts.json"), "w") as fh:
        json.dump(report, fh, indent=2)


if __name__ == "__main__":
    main(sys.argv[1:] or ["zone", "housing", "louver", "rod"])
