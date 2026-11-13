"""Phase 7a - product-level CAD of the SMA brake-duct actuator module (SOLIDWORKS).

Adds production detail to the Phase 1 duct (housing, 4 louvers, linkage rod):
  SMA spring       x6, 0.5 mm NiTiCu wire, 2.5 mm mean coil, 19 turns, black high-emissivity coating
  bias spring      steel, 0.8 mm wire, 5 mm mean coil, 14 turns
  crosshead        joins the six springs to the top of the linkage rod
  top bracket      spring anchor on the housing roof;  bottom bracket: bias-spring anchor under the floor
  flange           inlet mounting flange, 4 x M3
  screws           M3 x 10 socket head (x4)
Frame: Phase 1 assembly frame (mm). X = throat axis, Y = up, Z = louver shaft axis; linkage on +Z.
Pose: louvers at THETA (default 80 deg). SPRING_H / BIAS_H are the coil lengths at that pose.
Writes cad/product/*.SLDPRT, SMA_Duct_Module.SLDASM, STEP, renders, product_layout.json.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import math
import os

from rig_cad import MM, box, colour, cyl, subtract
from swlib import SW, byref_long, call, dbl_array, get, nothing

ROOT = PROJECT_ROOT
OUT = os.path.join(ROOT, "cad", "product")
P1 = os.path.join(ROOT, "cad")

CRANK_R, PHI0 = 10.89, math.radians(-40.0)
ROD_X0, ROD_Y0, ROD_Z = 66.34, 133.0, 35.5      # rod origin with louvers closed (0 deg)
SPRING_Z = 36.75
SPRING_X = [ROD_X0 - 10 + 4 * k for k in range(6)]
TOP_Y = 244.0                                     # SMA springs hang from the top bracket
BIAS_Y0 = 70.0                                    # bias spring sits on the bottom bracket

MATERIALS = {   # rgb, then ambient, diffuse, specular, shininess, transparency, emission
    "carbon": (0.13, 0.13, 0.15, 0.6, 0.8, 0.5, 0.6),
    "titanium": (0.62, 0.62, 0.66, 0.5, 0.8, 0.8, 0.7),
    "steel": (0.72, 0.72, 0.74, 0.5, 0.8, 0.9, 0.8),
    "niti_black": (0.06, 0.06, 0.07, 0.4, 0.9, 0.3, 0.3),
    "anodised": (0.85, 0.38, 0.05, 0.5, 0.8, 0.6, 0.6),
    "aluminium": (0.78, 0.79, 0.81, 0.5, 0.8, 0.7, 0.6),
    "black_oxide": (0.09, 0.09, 0.1, 0.4, 0.8, 0.6, 0.6),
}


def props(name):
    r, g, b, amb, dif, spec, shin = MATERIALS[name]
    return [r, g, b, amb, dif, spec, shin, 0.0, 0.0]


def rod_origin(theta_deg):
    th = math.radians(theta_deg)
    return (ROD_X0 + CRANK_R * (math.cos(th + PHI0) - math.cos(PHI0)),
            ROD_Y0 + CRANK_R * (math.sin(th + PHI0) - math.sin(PHI0)), ROD_Z)


def louver_rot(theta_deg):
    th = math.radians(theta_deg)
    return (0.0, 0.0, 1.0, -math.sin(th), math.cos(th), 0.0, -math.cos(th), -math.sin(th), 0.0)


def spring_lengths(theta_deg):
    _, ry, _ = rod_origin(theta_deg)
    crosshead_top = ry + 43.0 + 4.0              # rod top = origin + 43 mm, crosshead 4 mm thick
    rod_bottom = ry - 43.0
    return TOP_Y - crosshead_top, rod_bottom - BIAS_Y0


# ---------------------------------------------------------------- parts
def helix_spring(p, coil_r_mm, wire_d_mm, turns, length_mm):
    """Coil whose wire occupies y = 0..length_mm: the helix runs d/2..L-d/2 (the swept ends are round)."""
    height_mm = length_mm - wire_d_mm
    sk = p.sketch_on(p.plane(2))                  # Top plane: axis along +Y (wire spans -d/2..L-d/2)
    p.circles(sk, [(0, 0, 0)], coil_r_mm * MM)
    p.sm.InsertSketch(True)
    skf = p.last_feature("ProfileFeature")
    p.doc.ClearSelection2(True)
    skf.Select2(False, 0)
    call(p.doc, "InsertHelix", False, True, False, False, 0, height_mm * MM, height_mm * MM / turns, float(turns),
         0.0, 0.0)
    helix = p.last_feature("Helix")
    helix.Name = "Coil_Helix"
    p.doc.ClearSelection2(True)
    helix.Select2(False, 4)
    feat = call(p.fm, "InsertProtrusionSwept4", False, False, 0, False, False, 0, 0, False, 0.0, 0.0, 0, 0, True,
                True, True, 0.0, True, True, wire_d_mm * MM, 0)
    if feat is None:
        raise RuntimeError("spring sweep failed")
    feat.Name = "Coil_Wire"
    p.doc.ClearSelection2(True)
    helix.Select2(False, 0)
    get(p.doc, "BlankRefGeom")                    # hide the helix curve
    p.doc.ClearSelection2(True)


def sma_spring(p, height=None):
    helix_spring(p, 1.25, 0.5, 19, height or spring_lengths(80.0)[0])
    return "niti_black"


def bias_spring(p, height=None):
    helix_spring(p, 2.5, 0.8, 14, height or spring_lengths(80.0)[1])
    return "steel"


def crosshead(p):
    # modelled at the closed pose (rod top at y = ROD_Y0 + 43); placed by translation in the assembly
    y0 = ROD_Y0 + 43.0
    plate = box(p, 54, 79, y0, y0 + 4, 34.5, 39, "crosshead")
    slot = box(p, ROD_X0 - 3.1, ROD_X0 + 3.1, y0 - 0.1, y0 + 1.5, ROD_Z - 0.1, ROD_Z + 2.1, "rod_slot")
    subtract(p, plate, [slot], "rod slot")
    return "anodised"


def top_bracket(p):
    col = box(p, 46, 50, 192, TOP_Y + 4, 24, 40, "column")
    box(p, 46, 79, TOP_Y, TOP_Y + 4, 34.5, 39, "anchor_plate")
    return "anodised"


def bottom_bracket(p):
    box(p, 46, 50, BIAS_Y0 - 4, 88, 24, 40, "column")
    box(p, 46, 77, BIAS_Y0 - 4, BIAS_Y0, 34.5, 39, "anchor_plate")
    return "anodised"


def flange(p):
    from phase2_domain_cad import rect_yz
    box(p, 98, 101, 73, 207, -47, 47, "flange")
    sk = p.sketch_on(p.plane(3))                 # Right plane (normal X): port + 4 bolt holes, cut through all
    rect_yz(p, sk, 90 * MM, 190 * MM, -30 * MM, 30 * MM)
    p.circles(sk, [(0, y * MM, z * MM) for y, z in ((80, -40), (80, 40), (200, -40), (200, 40))], 1.7 * MM)
    feat = p.close_sketch()
    for flip in (False, True):
        p.doc.ClearSelection2(True)
        feat.Select2(False, 0)
        cut = p.fm.FeatureCut4(False, flip, False, 1, 1, 0.0, 0.0, False, False, False, False, 0.0, 0.0, False,
                               False, False, False, False, True, True, False, False, False, 0, 0.0, False, False)
        if cut is not None:
            cut.Name = "Port_and_M3_holes"
            break
    else:
        raise RuntimeError("flange cut failed")
    faces = len(get(p.bodies()[0], "GetFaces"))
    if faces < 6 + 4 + 4:                         # plate 6 + port 4 + 4 hole walls
        raise RuntimeError(f"flange has only {faces} faces: holes missing")
    return "aluminium"


def screw(p):
    cyl(p, "x", (0, 0), 2.75, 3.0, 6.0, "head")
    cyl(p, "x", (0, 0), 1.5, -7.0, 3.0, "shank")
    return "black_oxide"


NEW_PARTS = [("SMA_Spring_NiTiCu_0p5", sma_spring), ("Bias_Spring_Steel", bias_spring),
             ("Rod_Crosshead", crosshead), ("Spring_Bracket_Top", top_bracket),
             ("Spring_Bracket_Bottom", bottom_bracket), ("Inlet_Flange", flange), ("Screw_M3x10", screw)]
P1_COLOURS = {"Brake_Duct_Housing": "carbon", "Nitinol_Louver_Flap": "titanium", "Control_Linkage_Rod": "steel"}
ROT_ID = (1, 0, 0, 0, 1, 0, 0, 0, 1)


def placements(theta=80.0):
    """[(file stem, rotation, translation mm)] for every component at louver angle theta."""
    rx, ry, rz = rod_origin(theta)
    dx, dy = rx - ROD_X0, ry - ROD_Y0
    out = [("Brake_Duct_Housing", ROT_ID, (0, 0, 0))]
    out += [("Nitinol_Louver_Flap", louver_rot(theta), (58, y, 0)) for y in (110, 130, 150, 170)]
    out += [("Control_Linkage_Rod", ROT_ID, (rx, ry, rz)), ("Rod_Crosshead", ROT_ID, (dx, dy, 0)),
            ("Spring_Bracket_Top", ROT_ID, (0, 0, 0)), ("Spring_Bracket_Bottom", ROT_ID, (0, 0, 0)),
            ("Inlet_Flange", ROT_ID, (0, 0, 0))]
    top = ROD_Y0 + 43.0 + 4.0 + dy
    out += [("SMA_Spring_NiTiCu_0p5", ROT_ID, (x + dx, top + 0.25, SPRING_Z)) for x in SPRING_X]   # + wire radius
    out += [("Bias_Spring_Steel", ROT_ID, (ROD_X0 + dx, BIAS_Y0 + 0.4, SPRING_Z))]
    out += [("Screw_M3x10", ROT_ID, (101, y, z)) for y, z in ((80, -40), (80, 40), (200, -40), (200, 40))]
    return out


def part_path(stem):
    return os.path.join(OUT if not any(stem == k for k in P1_COLOURS) else P1, stem + ".SLDPRT")


def build_parts(sw):
    for name, fn in NEW_PARTS:
        path = part_path(name)
        sw.close_if_open(path)
        p = sw.new_part()
        mat = fn(p)
        p.doc.MaterialPropertyValues = dbl_array(props(mat))
        p.rebuild()
        p.view_iso()
        p.save(path)
        sw.app.CloseDoc(get(p.doc, "GetTitle"))
        print("PART", name)


def assemble(sw, theta=80.0, asm_name="SMA_Duct_Module"):
    doc = sw.new_assembly()
    comps = []
    for stem, rot, t in placements(theta):
        path = part_path(stem)
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
        if stem in P1_COLOURS:
            call(comp, "SetMaterialPropertyValues2", dbl_array(props(P1_COLOURS[stem])), 2, nothing())
        comps.append((stem, comp))
    doc.ForceRebuild3(False)
    sw.app.ActivateDoc3(get(doc, "GetTitle"), False, 0, byref_long())
    return doc, comps


def interferences(doc):
    idm = get(doc, "InterferenceDetectionManager")
    idm.TreatCoincidenceAsInterference = False
    idm.IncludeMultibodyPartInterferences = False
    out = []
    for it in get(idm, "GetInterferences") or ():
        out.append({"components": [get(c, "Name2") for c in get(it, "Components")],
                    "volume_mm3": round(get(it, "Volume") * 1e9, 3)})
    idm.Done()
    return out


def render(doc, path, view="*Isometric", zoom_to=None):
    ext = doc.Extension
    doc.ShowNamedView2(view, -1)
    get(doc, "ViewZoomtofit2")
    doc.GraphicsRedraw2()
    err, warn = byref_long(), byref_long()
    ext.SaveAs(path, 0, 1 | 2, nothing(), err, warn)


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    sw.app.CloseAllDocuments(True)
    build_parts(sw)
    doc, comps = assemble(sw, 80.0)
    clashes = interferences(doc)
    print("INTERFERENCES", json.dumps(clashes))
    path = os.path.join(OUT, "SMA_Duct_Module.SLDASM")
    err, warn = byref_long(), byref_long()
    doc.ShowNamedView2("*Isometric", 7)
    get(doc, "ViewZoomtofit2")
    if not doc.Extension.SaveAs(path, 0, 1, nothing(), err, warn):
        raise RuntimeError(f"save failed {err.value}")
    doc.Extension.SaveAs(os.path.join(OUT, "SMA_Duct_Module.STEP"), 0, 1 | 2, nothing(), err, warn)
    for view, fname in (("*Isometric", "Module_iso.png"), ("*Trimetric", "Module_trimetric.png"),
                        ("*Front", "Module_front.png"), ("*Right", "Module_right.png")):
        render(doc, os.path.join(OUT, fname), view)
    json.dump({"theta_deg": 80.0, "spring_len_mm": spring_lengths(80.0), "interferences": clashes,
               "components": [s for s, _ in comps]}, open(os.path.join(OUT, "product_layout.json"), "w"), indent=1)
    print("SAVED", path, "springs", [round(v, 2) for v in spring_lengths(80.0)])


if __name__ == "__main__":
    main()
