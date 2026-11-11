"""Phase 2 - build the CHT flow domain as a multi-body SOLIDWORKS part and export Parasolid.

Bodies (names become Fluent regions):
  fluid_domain  fluid  box x[-150,160] y[50,250] z[-70,70] mm minus every other body
  sma_pocket    fluid  air around the SMA spring, x[-33,-16] y[100,129] z[-6.5,6.5] mm (rotor wake)
  rotor         solid  carbon-carbon rotor segment inside the box (|x| <= 16 mm, r <= 164 mm)
  duct_wall     solid  CFRP throat tube, 104 x 64 outer / 100 x 60 inner, x[18,98] mm
  louver_1..4   solid  closed louvers, 2 x 18 x 56 mm at x = 58 mm, 20 mm pitch
Named faces on fluid_domain: inlet (x = +160 mm, inboard), outlet (x = -150 mm), farfield (rest).
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import math
import os

from swlib import END_MID_PLANE, START_OFFSET, SW, call, disp_array, get, nothing

OUT = PROJECT_ROOT + r"\cfd"
BOX = {"x": (-0.150, 0.160), "y": (0.050, 0.250), "z": (-0.070, 0.070)}   # outlet moved 100 mm further back
ROTOR_R, ROTOR_T = 0.164, 0.032
H_CY, H_H, H_W, WALL, H_X0, H_LEN = 0.140, 0.100, 0.060, 0.002, 0.018, 0.080
L_X, L_T, L_W, L_L, PITCH = 0.058, 0.002, 0.018, 0.056, 0.020
# Relocated SMA pocket: flush on the rotor's outboard (wake) face at mid-height.
# Baseline position was x[58,75] y[176,205] z[32,45] mm beside the duct (pocket air stayed at 300 K).
OPEN = os.environ.get("LOUVERS", "closed").lower() == "open"   # open = 90 deg (MAPDL hot state 86 deg)
SUFFIX = os.environ.get("SUFFIX", "")
POCKET = {"x": (-0.033, -0.016), "y": (0.100, 0.129), "z": (-0.0065, 0.0065)}
SWBODYCUT = 15902


def rect_yz(p, sk, y0, y1, z0, z1):
    p.center_rect(sk, (0, (y0 + y1) / 2, (z0 + z1) / 2), (0, y1, z1))


def x_extrude(p, sk_feat, x0, x1, name):
    """Extrude a Right-plane sketch to occupy x0..x1 as a separate body."""
    if x0 < 0:
        return p.extrude(sk_feat, x1 - x0, merge=False, start=START_OFFSET, start_offset=-x0, flip_start=True,
                         name=name)
    return p.extrude(sk_feat, x1 - x0, merge=False, start=START_OFFSET, start_offset=x0, name=name)


def make_solids(p, tag):
    """Create rotor, duct wall, louvers and pocket bodies; return their feature names."""
    right = p.plane(3)
    # rotor segment clipped to the box
    sk = p.sketch_on(right)
    y0 = BOX["y"][0]
    zc = BOX["z"][1]
    yc = math.sqrt(ROTOR_R ** 2 - zc ** 2)
    pts = [(0, y0, -zc), (0, y0, zc), (0, yc, zc)]
    s = [p.to_sketch(sk, *q) for q in pts]
    s_end = p.to_sketch(sk, 0, yc, -zc)
    s_mid = p.to_sketch(sk, 0, ROTOR_R, 0)
    p.sm.AddToDB = True
    p.sm.CreateLine(s[0][0], s[0][1], 0, s[1][0], s[1][1], 0)
    p.sm.CreateLine(s[1][0], s[1][1], 0, s[2][0], s[2][1], 0)
    call(p.sm, "Create3PointArc", s[2][0], s[2][1], 0, s_end[0], s_end[1], 0, s_mid[0], s_mid[1], 0)
    p.sm.CreateLine(s_end[0], s_end[1], 0, s[0][0], s[0][1], 0)
    p.sm.AddToDB = False
    p.extrude(p.close_sketch(), ROTOR_T, end=END_MID_PLANE, merge=False, name=f"rotor_{tag}")
    # duct wall tube
    sk = p.sketch_on(right)
    rect_yz(p, sk, H_CY - H_H / 2 - WALL, H_CY + H_H / 2 + WALL, -H_W / 2 - WALL, H_W / 2 + WALL)
    rect_yz(p, sk, H_CY - H_H / 2, H_CY + H_H / 2, -H_W / 2, H_W / 2)
    x_extrude(p, p.close_sketch(), H_X0, H_X0 + H_LEN, f"duct_{tag}")
    # louvers (closed)
    for k in range(4):
        yk = H_CY + (k - 1.5) * PITCH
        sk = p.sketch_on(right)
        if OPEN:
            rect_yz(p, sk, yk - L_T / 2, yk + L_T / 2, -L_L / 2, L_L / 2)
            x_extrude(p, p.close_sketch(), L_X - L_W / 2, L_X + L_W / 2, f"louver{k + 1}_{tag}")
        else:
            rect_yz(p, sk, yk - L_W / 2, yk + L_W / 2, -L_L / 2, L_L / 2)
            x_extrude(p, p.close_sketch(), L_X - L_T / 2, L_X + L_T / 2, f"louver{k + 1}_{tag}")
    # SMA pocket
    sk = p.sketch_on(right)
    rect_yz(p, sk, *POCKET["y"], *POCKET["z"])
    x_extrude(p, p.close_sketch(), *POCKET["x"], f"pocket_{tag}")


def classify(body):
    b = get(body, "GetBodyBox")
    x0, y0, z0, x1, y1, z1 = b
    if abs(x0 - BOX["x"][0]) < 1e-6 and abs(x1 - BOX["x"][1]) < 1e-6:
        return "fluid_domain"
    if abs(x0 - POCKET["x"][0]) < 1e-6 and abs(z0 - POCKET["z"][0]) < 1e-6:
        return "sma_pocket"
    if abs(x0 + ROTOR_T / 2) < 1e-6:
        return "rotor"
    if abs(x0 - H_X0) < 1e-6:
        return "duct_wall"
    if abs(x0 - (L_X - (L_W if OPEN else L_T) / 2)) < 1e-6:
        k = int(round(((y0 + y1) / 2 - H_CY) / PITCH + 1.5)) + 1
        return f"louver_{k}"
    raise RuntimeError(f"unclassified body box {b}")


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    sw.close_if_open(os.path.join(OUT, f"Phase2_CHT_Domain{SUFFIX}.SLDPRT"))
    p = sw.new_part()
    sk = p.sketch_on(p.plane(3))
    rect_yz(p, sk, *BOX["y"], *BOX["z"])
    x_extrude(p, p.close_sketch(), *BOX["x"], "fluid_box")
    make_solids(p, "tool")
    bodies = p.bodies()
    fluid = [b for b in bodies if classify(b) == "fluid_domain"][0]
    tools = [b for b in bodies if b is not fluid and classify(b) != "fluid_domain"]
    p.doc.ClearSelection2(True)
    combine = call(p.fm, "InsertCombineFeature", SWBODYCUT, fluid, disp_array(tools))
    if combine is None:
        raise RuntimeError("combine subtract failed")
    p.rebuild()
    if len(p.bodies()) != 1:
        raise RuntimeError(f"after subtract expected 1 body, got {len(p.bodies())}")
    make_solids(p, "keep")
    p.rebuild()
    names = {}
    for body in p.bodies():
        nm = classify(body)
        body.Name = nm
        names[nm] = [round(v * 1000, 2) for v in get(body, "GetBodyBox")]
    if len(names) != 8:
        raise RuntimeError(f"expected 8 named bodies, got {sorted(names)}")
    # name boundary faces on the fluid body
    fluid = [b for b in p.bodies() if b.Name == "fluid_domain"][0]
    named = {"inlet": 0, "outlet": 0, "farfield": 0}
    for face in get(fluid, "GetFaces"):
        box = get(face, "GetBox")
        x0, y0, z0, x1, y1, z1 = box
        label = None
        if abs(x0 - BOX["x"][1]) < 1e-6 and abs(x1 - BOX["x"][1]) < 1e-6:
            label = "inlet"
        elif abs(x0 - BOX["x"][0]) < 1e-6 and abs(x1 - BOX["x"][0]) < 1e-6:
            label = "outlet"
        elif (abs(y0 - y1) < 1e-6 and (abs(y0 - BOX["y"][0]) < 1e-6 or abs(y0 - BOX["y"][1]) < 1e-6)) or \
                (abs(z0 - z1) < 1e-6 and abs(abs(z0) - BOX["z"][1]) < 1e-6):
            label = "farfield"
        if label:
            ok = call(p.doc, "SetEntityName", face, label if label != "farfield" else f"farfield{named['farfield']}")
            named[label] += 1 if ok else 0
    # tag every non-fluid body with one named face so Fluent can identify it
    for body in p.bodies():
        if body.Name == "fluid_domain":
            continue
        face = get(body, "GetFaces")[0]
        if not call(p.doc, "SetEntityName", face, f"id_{body.Name}"):
            raise RuntimeError(f"could not tag {body.Name}")
    print("bodies:", json.dumps(names))
    print("named faces:", named)
    p.view_iso()
    path = os.path.join(OUT, f"Phase2_CHT_Domain{SUFFIX}.SLDPRT")
    p.save(path)
    p.export(os.path.join(OUT, f"Phase2_CHT_Domain{SUFFIX}.x_t"))
    p.export(os.path.join(OUT, f"Phase2_CHT_Domain{SUFFIX}.png"))
    with open(os.path.join(OUT, f"phase2_domain{SUFFIX}.json"), "w") as fh:
        json.dump({"bodies": names, "named_faces": named, "box_m": BOX, "pocket_m": POCKET}, fh, indent=2)


if __name__ == "__main__":
    main()
