"""EV re-target - SOLIDWORKS model of the passive SMA grille-shutter module (6 louvers, scotch-yoke drive).

Frame (mm): X = flow direction (air enters at x = 0), Y = up, Z = across the car. Drive side on +Z.
  frame          glass-filled PA66, 490 x 390 outside, 40 deep, opening 450 x 350 (matches the radiator core)
  louvers x6     58 mm chord, 448 mm span, 3 mm thick, pivots on x = 20 at a 58.3 mm pitch; each carries a 15 mm
                 crank with a 4 mm pin on the drive side
  yoke bar       aluminium; horizontal slots take the six crank pins, so the bar moves straight up as the
                 louvers open: lift = 15 sin(theta), 14.8 mm at the full 80 deg
  SMA pack       5 NiTi springs (1.0 mm wire, 5 mm coil) in a clear coolant sleeve on the coolant return line.
                 Warm coolant (As 35 / Af 40 C) makes them contract and lift the bar -> louvers open
  bias spring    steel tension spring under the bar: pulls the louvers shut when the coolant cools
Builds the parts, the assembly at theta = 80 deg (open) and 0 deg (closed), runs interference detection on both,
and writes ev/cad/EV_Shutter_{Open,Closed}.SLDASM, STEP, renders and ev_shutter_layout.json.
"""
from __future__ import annotations

import json
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from product_cad import helix_spring, interferences, render  # noqa: E402
from rig_cad import MM, box, cyl, subtract  # noqa: E402
from swlib import SW, byref_long, call, dbl_array, get, nothing  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cad")
W_OUT, H_OUT, DEPTH, WALL = 490.0, 390.0, 40.0, 20.0
PIV_X = 20.0
N_LOUV = 6
PITCH = (H_OUT - 2 * WALL) / N_LOUV
PIV_Y = [WALL + PITCH * (k + 0.5) for k in range(N_LOUV)]
CHORD, T_LOUV, CRANK = 58.0, 3.0, 15.0
PIN_R, CPIN_R = 3.0, 2.0
AX_X, AX_Z = 29.0, 498.0                      # actuator axis (sleeve, springs, bias spring)
BAR_Y0, BAR_Y1 = 28.0, 400.0                  # yoke bar span at the closed pose (bottom clears the cranks)
TOP = 480.0                                   # SMA springs hang from the sleeve cap
BIAS_ANCHOR = -9.0
SMA_XZ = [(21, 492), (37, 492), (29, 498), (21, 504), (37, 504)]
SMA_WIRE, SMA_R, SMA_TURNS = 1.0, 2.5, 16
BIAS_WIRE, BIAS_R, BIAS_TURNS = 1.2, 4.0, 18

MATERIALS = {   # rgb, ambient, diffuse, specular, shininess, transparency
    "pa66_black": (0.10, 0.10, 0.11, 0.5, 0.8, 0.3, 0.3, 0.0),
    "louver_grey": (0.30, 0.31, 0.33, 0.5, 0.8, 0.5, 0.5, 0.0),
    "aluminium": (0.78, 0.79, 0.81, 0.5, 0.8, 0.7, 0.6, 0.0),
    "anodised": (0.85, 0.38, 0.05, 0.5, 0.8, 0.6, 0.6, 0.0),
    "niti_black": (0.06, 0.06, 0.07, 0.4, 0.9, 0.3, 0.3, 0.0),
    "steel": (0.72, 0.72, 0.74, 0.5, 0.8, 0.9, 0.8, 0.0),
    "clear_pc": (0.55, 0.75, 0.95, 0.5, 0.8, 0.9, 0.9, 0.65),
}


def props(name):
    r, g, b, amb, dif, spec, shin, tr = MATERIALS[name]
    return [r, g, b, amb, dif, spec, shin, tr, 0.0]


def lift(theta):
    return CRANK * math.sin(math.radians(theta))


def sma_len(theta):
    return TOP - (BAR_Y1 + 4.0 + lift(theta))


def bias_len(theta):
    return BAR_Y0 + lift(theta) - BIAS_ANCHOR


# ---------------------------------------------------------------- parts
def frame(p):
    # each tool is cut as soon as it exists: handles to earlier tool bodies go stale and get skipped
    body = box(p, 0, DEPTH, 0, H_OUT, 0, W_OUT, "frame")
    body = subtract(p, body, [box(p, -1, DEPTH + 1, WALL, H_OUT - WALL, WALL, W_OUT - WALL, "opening")], "opening")
    for k, y in enumerate(PIV_Y):
        body = subtract(p, body, [cyl(p, "z", (PIV_X, y), PIN_R + 0.3, -1, W_OUT + 1, f"pivot_{k}")], f"bore {k}")
    if len(p.bodies()) != 1:
        raise RuntimeError(f"frame has {len(p.bodies())} bodies")
    return "pa66_black"


def louver(p):
    """Modelled closed with the pivot at the origin: chord along Y, crank along +X on the drive side."""
    # crank first: a sketch on the x = 0 plane fails when it lands on existing edges
    box(p, 0, CRANK, -4, 4, 491, 495, "crank")
    box(p, -T_LOUV / 2, T_LOUV / 2, -CHORD / 2, CHORD / 2, WALL + 1, W_OUT - WALL - 1, "blade")
    cyl(p, "z", (0, 0), PIN_R, 2, 495, "pivot_pin")
    cyl(p, "z", (CRANK, 0), CPIN_R, 495, 503, "crank_pin")
    return "louver_grey"


def yoke_bar(p):
    # small slot sketches first: after the long bar exists they fail on the x = 0 sketch plane
    x0, x1 = PIV_X + CRANK * math.cos(math.radians(80)) - CPIN_R - 0.3, PIV_X + CRANK + CPIN_R + 0.3
    box(p, x0, x1, PIV_Y[0] - CPIN_R - 0.3, PIV_Y[0] + CPIN_R + 0.3, 495, 501, "slot_0")
    bar = box(p, 17, 41, BAR_Y0, BAR_Y1, 496, 500, "bar")
    slot_box = [b for b in p.bodies() if get(b, "GetBodyBox")[4] / MM < 100][0]
    bar = subtract(p, [b for b in p.bodies() if b is not slot_box][0], [slot_box], "slot 0")
    for k, y in enumerate(PIV_Y[1:], 1):
        tool = cyl(p, "z", (x0 + CPIN_R + 0.3, y), CPIN_R + 0.3, 495, 501, f"slot_end_{k}")
        bar = subtract(p, bar, [tool], f"slot end {k}")
        tool = cyl(p, "z", (x1 - CPIN_R - 0.3, y), CPIN_R + 0.3, 495, 501, f"slot_end_b_{k}")
        bar = subtract(p, bar, [tool], f"slot end b {k}")
        sk = p.sketch_on(p.plane(1))                 # Front plane (XY): slot web as a rectangle, cut through
        p.center_rect(sk, ((x0 + x1) / 2 * MM, y * MM, 0), ((x1 - CPIN_R - 0.3) * MM, (y + CPIN_R + 0.3) * MM, 0))
        feat = p.close_sketch()
        for flip in (False, True):
            p.doc.ClearSelection2(True)
            feat.Select2(False, 0)
            cut = p.fm.FeatureCut4(False, flip, False, 1, 1, 0.0, 0.0, False, False, False, False, 0.0, 0.0, False,
                                   False, False, False, False, True, True, False, False, False, 0, 0.0, False, False)
            if cut is not None:
                break
        else:
            raise RuntimeError(f"slot web {k} failed")
        bar = p.bodies()[0]
    if len(p.bodies()) != 1:
        raise RuntimeError(f"yoke bar has {len(p.bodies())} bodies")
    cyl(p, "y", (AX_X, AX_Z), 12.0, BAR_Y1, BAR_Y1 + 4, "crosshead")
    return "aluminium"


def sleeve(p):
    tube = cyl(p, "y", (AX_X, AX_Z), 17.0, 410, TOP, "sleeve")
    subtract(p, tube, [cyl(p, "y", (AX_X, AX_Z), 15.0, 409, TOP + 1, "bore")], "sleeve bore")
    cyl(p, "y", (AX_X, AX_Z), 17.0, TOP, TOP + 6, "cap")
    for y in (425.0, 468.0):
        s = cyl(p, "x", (y, AX_Z), 4.0, AX_X + 15.5, AX_X + 32, f"spigot_{y:.0f}")
        subtract(p, s, [cyl(p, "x", (y, AX_Z), 2.5, AX_X + 15, AX_X + 33, "spigot_bore")], "spigot bore")
    return "clear_pc"


def top_bracket(p):
    box(p, 0, 12, H_OUT, TOP + 6, 484, 512, "upright")
    box(p, 0, DEPTH, H_OUT, H_OUT + 4, 470, 484, "foot")
    return "anodised"


def bottom_bracket(p):
    box(p, 17, 41, BIAS_ANCHOR - 4, BIAS_ANCHOR, 491, 505, "anchor_plate")
    box(p, 0, 17, BIAS_ANCHOR - 4, 0, 491, 505, "column")
    box(p, 0, DEPTH, -4, 0, 470, 491, "foot")
    return "anodised"


def spring_part(wire, coil_r, turns, length, mat):
    def build(p):
        helix_spring(p, coil_r, wire, turns, length)
        return mat
    return build


def part_list():
    parts = [("EV_Shutter_Frame", frame), ("EV_Louver", louver), ("EV_Yoke_Bar", yoke_bar),
             ("EV_Coolant_Sleeve", sleeve), ("EV_Actuator_Bracket", top_bracket),
             ("EV_Bias_Anchor", bottom_bracket)]
    for th, tag in ((80.0, "open"), (0.0, "closed")):
        parts.append((f"EV_SMA_Spring_{tag}", spring_part(SMA_WIRE, SMA_R, SMA_TURNS, sma_len(th), "niti_black")))
        parts.append((f"EV_Bias_Spring_{tag}", spring_part(BIAS_WIRE, BIAS_R, BIAS_TURNS, bias_len(th), "steel")))
    return parts


ROT_ID = (1, 0, 0, 0, 1, 0, 0, 0, 1)


def rot_z(theta):
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    return (c, s, 0, -s, c, 0, 0, 0, 1)


def placements(theta):
    tag = "open" if theta > 40 else "closed"
    dy = lift(theta)
    out = [("EV_Shutter_Frame", ROT_ID, (0, 0, 0)), ("EV_Actuator_Bracket", ROT_ID, (0, 0, 0)),
           ("EV_Bias_Anchor", ROT_ID, (0, 0, 0)), ("EV_Coolant_Sleeve", ROT_ID, (0, 0, 0))]
    out += [("EV_Louver", rot_z(theta), (PIV_X, y, 0)) for y in PIV_Y]
    out += [("EV_Yoke_Bar", ROT_ID, (0, dy, 0))]
    out += [(f"EV_SMA_Spring_{tag}", ROT_ID, (x, BAR_Y1 + 4 + dy + SMA_WIRE / 2, z)) for x, z in SMA_XZ]
    out += [(f"EV_Bias_Spring_{tag}", ROT_ID, (AX_X, BIAS_ANCHOR + BIAS_WIRE / 2, AX_Z))]
    return out


def build_parts(sw):
    for name, fn in part_list():
        path = os.path.join(OUT, name + ".SLDPRT")
        sw.close_if_open(path)
        p = sw.new_part()
        mat = fn(p)
        p.doc.MaterialPropertyValues = dbl_array(props(mat))
        p.rebuild()
        p.view_iso()
        p.save(path)
        sw.app.CloseDoc(get(p.doc, "GetTitle"))
        print("PART", name, flush=True)


def assemble(sw, theta):
    doc = sw.new_assembly()
    for stem, rot, t in placements(theta):
        path = os.path.join(OUT, stem + ".SLDPRT")
        err, warn = byref_long(), byref_long()
        sw.app.OpenDoc6(path, 1, 1, "", err, warn)
        sw.app.ActivateDoc3(get(doc, "GetTitle"), False, 0, byref_long())
        comp = call(doc, "AddComponent5", path, 0, "", False, "", 0.0, 0.0, 0.0)
        if comp is None:
            raise RuntimeError(f"insert failed {path}")
        comp.Transform2 = call(sw.math, "CreateTransform", dbl_array(list(rot) + [v * MM for v in t] + [1.0, 0, 0, 0]))
        doc.ClearSelection2(True)
        call(comp, "Select4", False, nothing(), False)
        get(doc, "FixComponent")
        doc.ClearSelection2(True)
    doc.ForceRebuild3(False)
    sw.app.ActivateDoc3(get(doc, "GetTitle"), False, 0, byref_long())
    return doc


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    sw.app.CloseAllDocuments(True)
    if os.environ.get("SKIP_PARTS") != "1":
        build_parts(sw)
    layout = {"theta_open_deg": 80.0, "yoke_lift_mm": lift(80.0), "pivots_y_mm": PIV_Y,
              "sma_spring_len_mm": {"open": sma_len(80.0), "closed": sma_len(0.0)},
              "bias_spring_len_mm": {"open": bias_len(80.0), "closed": bias_len(0.0)}, "poses": {}}
    for theta, tag in ((80.0, "Open"), (0.0, "Closed")):
        doc = assemble(sw, theta)
        clashes = interferences(doc)
        print(tag, "INTERFERENCES", json.dumps(clashes), flush=True)
        path = os.path.join(OUT, f"EV_Shutter_{tag}.SLDASM")
        err, warn = byref_long(), byref_long()
        doc.ShowNamedView2("*Isometric", 7)
        get(doc, "ViewZoomtofit2")
        if not doc.Extension.SaveAs(path, 0, 1, nothing(), err, warn):
            raise RuntimeError(f"save failed {err.value}")
        doc.Extension.SaveAs(os.path.join(OUT, f"EV_Shutter_{tag}.STEP"), 0, 1 | 2, nothing(), err, warn)
        for view, fname in (("*Isometric", "iso"), ("*Trimetric", "trimetric"), ("*Front", "front"),
                            ("*Right", "right")):
            render(doc, os.path.join(OUT, f"EV_Shutter_{tag}_{fname}.png"), view)
        mp = get(doc.Extension, "CreateMassProperty")
        mass = None
        if mp is not None:
            mp.UseSystemUnits = True
            mass = float(get(mp, "Mass"))
        layout["poses"][tag] = {"theta_deg": theta, "interferences": clashes,
                                "components": len(placements(theta)), "mass_kg_default_density": mass}
        sw.app.CloseDoc(get(doc, "GetTitle"))
    json.dump(layout, open(os.path.join(OUT, "ev_shutter_layout.json"), "w"), indent=1)
    print("DONE", json.dumps(layout), flush=True)


if __name__ == "__main__":
    main()
