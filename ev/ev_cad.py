"""EV re-target - half-car CFD domain with a grille, a low-temperature radiator (porous zone) and an under-floor exit.

Frame (m): X along the car (nose at x = 0, air flows +X), Y up (road at y = 0), Z sideways (symmetry plane z = 0).
  car        4.4 m long, 1.8 m wide (half 0.9), 0.15-1.5 m high; side profile with r 0.3 m nose arcs and a 25 deg rear
             slant
  grille     opening 0.45 x 0.35 m (half 0.225 wide), y 0.47-0.82 m, in the flat nose face
  radiator   0.025 m thick at x 0.20-0.225 m, fills the grille duct (separate fluid region -> porous in Fluent)
  exit       duct turns down at x 0.30-0.45 m and leaves through the floor
  shutter    closed variant: the first 40 mm of the grille duct is solid (shutter blades closed)
Domain x -9 .. 22, y 0 .. 6, z 0 .. 4.5. Faces: inlet, outlet, ground, symmetry, farfield_*, car_*, id_radiator /
radiator_* (radiator body), radiator_if_* (fluid side of the radiator faces).
Run with SHUTTER=open|closed. Writes ev/cfd/EV_Front_{Open,Closed}.{SLDPRT,x_t,json,png}.
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from rig_cad import box, subtract  # noqa: E402
from swlib import SW, END_BLIND, call, dbl_array, get  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfd")
OPEN = os.environ.get("SHUTTER", "open").lower() == "open"
NAME = f"EV_Front_{'Open' if OPEN else 'Closed'}"
MM = 1e-3
DOM = dict(x=(-9000, 22000), y=(0, 6000), z=(0, 4500))
L, Y0, Y1, HALF_W, R_NOSE, SLANT_L = 4400, 150, 1500, 900, 300, 800
GR_Y, GR_Z = (470, 820), 225
RAD_X = (200, 225)
EXIT_X = (300, 450)


def side_profile(p):
    """Car side profile on the Front plane (XY), extruded 0..HALF_W along +Z."""
    sk = p.sketch_on(p.plane(1))
    yslant = Y1 - SLANT_L * math.tan(math.radians(25))
    pts = [(R_NOSE, Y0), (L, Y0), (L, yslant), (L - SLANT_L, Y1), (R_NOSE, Y1)]
    q = [p.to_sketch(sk, x * MM, y * MM, 0) for x, y in pts]
    a_bot_end = p.to_sketch(sk, 0, (Y0 + R_NOSE) * MM, 0)
    a_top_start = p.to_sketch(sk, 0, (Y1 - R_NOSE) * MM, 0)
    c_bot = p.to_sketch(sk, R_NOSE * MM, (Y0 + R_NOSE) * MM, 0)
    c_top = p.to_sketch(sk, R_NOSE * MM, (Y1 - R_NOSE) * MM, 0)
    sm = p.sm
    sm.AddToDB = True
    for a, b in zip(q, q[1:]):
        sm.CreateLine(a[0], a[1], 0, b[0], b[1], 0)
    # top-front arc: from (R, Y1) to (0, Y1-R), centre (R, Y1-R); bottom-front arc: (0, Y0+R) to (R, Y0)
    call(sm, "CreateArc", c_top[0], c_top[1], 0, q[4][0], q[4][1], 0, a_top_start[0], a_top_start[1], 0, 1)
    sm.CreateLine(a_top_start[0], a_top_start[1], 0, a_bot_end[0], a_bot_end[1], 0)
    call(sm, "CreateArc", c_bot[0], c_bot[1], 0, a_bot_end[0], a_bot_end[1], 0, q[0][0], q[0][1], 0, 1)
    sm.AddToDB = False
    feat = p.extrude(p.close_sketch(), HALF_W * MM, end=END_BLIND, merge=False, name="car_body")
    return feat


def corner_cut(p):
    """Round the plan-view front corner (Top plane XZ, cut through the car height)."""
    sk = p.sketch_on(p.plane(2))
    c = p.to_sketch(sk, R_NOSE * MM, 0, (HALF_W - R_NOSE) * MM)
    a0 = p.to_sketch(sk, 0, 0, (HALF_W - R_NOSE) * MM)
    a1 = p.to_sketch(sk, R_NOSE * MM, 0, HALF_W * MM)
    o0 = p.to_sketch(sk, -20 * MM, 0, (HALF_W - R_NOSE) * MM)
    o1 = p.to_sketch(sk, -20 * MM, 0, (HALF_W + 20) * MM)
    o2 = p.to_sketch(sk, R_NOSE * MM, 0, (HALF_W + 20) * MM)
    sm = p.sm
    sm.AddToDB = True
    sm.CreateLine(a0[0], a0[1], 0, o0[0], o0[1], 0)
    sm.CreateLine(o0[0], o0[1], 0, o1[0], o1[1], 0)
    sm.CreateLine(o1[0], o1[1], 0, o2[0], o2[1], 0)
    sm.CreateLine(o2[0], o2[1], 0, a1[0], a1[1], 0)
    call(sm, "CreateArc", c[0], c[1], 0, a0[0], a0[1], 0, a1[0], a1[1], 0, -1)
    sm.AddToDB = False
    feat = p.close_sketch()
    for flip in (False, True):
        p.doc.ClearSelection2(True)
        feat.Select2(False, 0)
        cut = p.fm.FeatureCut4(False, flip, False, 1, 1, 0.0, 0.0, False, False, False, False, 0.0, 0.0, False, False,
                               False, False, False, True, True, False, False, False, 0, 0.0, False, False)
        if cut is not None:
            return cut
    raise RuntimeError("corner cut failed")


def bodies_by_box(p):
    return {tuple(round(v / MM, 1) for v in get(b, "GetBodyBox")): b for b in p.bodies()}


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    sw.app.CloseAllDocuments(True)
    p = sw.new_part()
    side_profile(p)
    # (plan-view corner rounding removed: its tangent meeting with the nose arcs left a point the mesher's
    #  share-topology step cannot resolve; sharp side corners do not change the open-vs-closed comparison)
    car = p.bodies()[0]
    print("car box", [round(v / MM) for v in get(car, "GetBodyBox")])
    # grille duct and floor exit
    duct_x0 = -50 if OPEN else 40
    t1 = box(p, duct_x0, EXIT_X[1], GR_Y[0], GR_Y[1], -10, GR_Z, "grille_duct")
    car = subtract(p, car, [t1], "grille duct")
    t2 = box(p, EXIT_X[0], EXIT_X[1], Y0 - 50, GR_Y[0] + 5, -10, GR_Z, "floor_exit")
    car = subtract(p, car, [t2], "floor exit")
    car_box = [round(v / MM, 1) for v in get(car, "GetBodyBox")]
    # radiator: keep one copy as the porous fluid region, use a second as the cutting tool
    rad = box(p, *RAD_X, *GR_Y, 0, GR_Z, "radiator")
    rad_tool = box(p, *RAD_X, *GR_Y, 0, GR_Z, "radiator_tool")
    dom = box(p, *DOM["x"], *DOM["y"], *DOM["z"], "fluid_domain")
    dom = subtract(p, dom, [car], "car")
    rad_box = [RAD_X[0], GR_Y[0], 0, RAD_X[1], GR_Y[1], GR_Z]       # body handles go stale after a combine
    # subtract the radiator tool: find it as the radiator-sized body that is not the kept radiator
    same = [b for b in p.bodies()
            if all(abs(v / MM - r) < 0.5 for v, r in zip(get(b, "GetBodyBox"), rad_box))]
    if len(same) != 2:
        raise RuntimeError(f"expected two radiator boxes, found {len(same)}")
    # direct combine: the generic subtract() check would mistake the kept radiator for an uncut tool
    dom = [b for b in p.bodies() if get(b, "GetBodyBox")[3] / MM > 20000][0]
    p.doc.ClearSelection2(True)
    from swlib import disp_array  # noqa: E402
    if call(p.fm, "InsertCombineFeature", 15902, dom, disp_array([same[1]])) is None:
        raise RuntimeError("radiator tool cut failed")
    p.rebuild()
    bodies = p.bodies()
    print("bodies", len(bodies), [[round(v / MM) for v in get(b, "GetBodyBox")] for b in bodies])
    # open: fluid domain + radiator; closed: also the air pocket sealed between the shutter and the radiator
    if len(bodies) != (2 if OPEN else 3):
        raise RuntimeError("unexpected body count")
    fluid = [b for b in bodies if get(b, "GetBodyBox")[3] / MM > 20000][0]
    radb = [b for b in bodies if all(abs(v / MM - r) < 0.5 for v, r in zip(get(b, "GetBodyBox"), rad_box))][0]
    plenum = [b for b in bodies if b is not fluid and b is not radb]
    fluid.Name = "fluid_domain"
    radb.Name = "radiator"

    tol = 0.5
    def on(v, ref):
        return abs(v - ref) < tol

    labels, counts = {}, {}

    def name_face(face, part):
        i = counts.get(part, 0)
        counts[part] = i + 1
        lab = part if part in ("inlet", "outlet") else f"{part}_{i}"
        if not call(p.doc, "SetEntityName", face, lab):
            raise RuntimeError(f"could not name {lab}")
        labels.setdefault(part, []).append(lab)

    rx0, ry0, rz0, rx1, ry1, rz1 = RAD_X[0], GR_Y[0], 0, RAD_X[1], GR_Y[1], GR_Z
    for face in get(fluid, "GetFaces"):
        b = [v / MM for v in get(face, "GetBox")]
        x0, y0, z0, x1, y1, z1 = b
        if on(x0, DOM["x"][0]) and on(x1, DOM["x"][0]):
            part = "inlet"
        elif on(x0, DOM["x"][1]) and on(x1, DOM["x"][1]):
            part = "outlet"
        elif on(y0, 0) and on(y1, 0):
            part = "ground"
        elif on(z0, 0) and on(z1, 0):
            part = "symmetry"
        elif (on(y0, DOM["y"][1]) and on(y1, DOM["y"][1])) or (on(z0, DOM["z"][1]) and on(z1, DOM["z"][1])):
            part = "farfield"
        elif x0 >= rx0 - tol and x1 <= rx1 + tol and y0 >= ry0 - tol and y1 <= ry1 + tol and z1 <= rz1 + tol:
            part = "radiator_if"
        else:
            part = "car"
        name_face(face, part)
    first = True
    for face in get(radb, "GetFaces"):
        if first:
            if not call(p.doc, "SetEntityName", face, "id_radiator"):
                raise RuntimeError("id_radiator")
            labels.setdefault("radiator", []).append("id_radiator")
            first = False
        else:
            name_face(face, "radiator")
    for pb in plenum:
        pb.Name = "plenum"
        tagged = False
        for face in get(pb, "GetFaces"):
            x0, y0, z0, x1, y1, z1 = [v / MM for v in get(face, "GetBox")]
            if not tagged and not (on(x0, RAD_X[0]) and on(x1, RAD_X[0])):
                call(p.doc, "SetEntityName", face, "id_plenum")
                labels.setdefault("plenum", []).append("id_plenum")
                tagged = True
            elif on(x0, RAD_X[0]) and on(x1, RAD_X[0]):
                try:
                    name_face(face, "plenum_if")
                except RuntimeError as exc:
                    print("  (unnamed interface face)", exc)
            # other pocket faces stay unnamed: they become plain walls in Fluent
    print("FACES", {k: len(v) for k, v in labels.items()})
    p.view_iso()
    path = os.path.join(OUT, NAME + ".SLDPRT")
    p.save(path)
    p.export(os.path.join(OUT, NAME + ".x_t"))
    p.export(os.path.join(OUT, NAME + ".png"))
    json.dump({"open": OPEN, "domain_mm": DOM, "car_box_mm": car_box, "radiator_box_mm": rad_box, "labels": labels},
              open(os.path.join(OUT, NAME + ".json"), "w"), indent=1)
    print("SAVED", path)


if __name__ == "__main__":
    main()
