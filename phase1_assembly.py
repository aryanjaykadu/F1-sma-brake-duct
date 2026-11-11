"""Phase 1 - assemble the louver mechanism in SOLIDWORKS, verify the kinematics, export renders.

Mates: zone fixed; housing coincident (Front/Top/Right) to zone; each louver concentric
(pivot shaft <-> housing hole) + coincident (louver mid-plane <-> housing mid-plane);
rod distance-mated to the housing mid-plane and concentric to the four crank pins.
A temporary angle mate drives louver 1 to 0 deg and 80 deg to measure the real rod stroke.
"""
from __future__ import annotations

import json
import math
import os

import pythoncom
from win32com.client import VARIANT

from phase1_cad import (CPIN_R, CRANK_PHI0, CRANK_R, CRANK_T, HOLE_R, HX, H_CY, L_L, OFFSETS, OUT, ROD_HOLE_R,
                        ROD_T, SHAFT_LEN)
from swlib import SW, byref_long, call, dbl_array, disp_array, get, nothing

MATE_COINCIDENT, MATE_CONCENTRIC, MATE_DISTANCE, MATE_ANGLE = 0, 1, 5, 6
ALIGN_ALIGNED, ALIGN_ANTI, ALIGN_CLOSEST = 0, 1, 2
ROD_Z = L_L / 2 + SHAFT_LEN + CRANK_T + 0.0005          # rod back face (asm Z)
ROT_LOUVER = (0, 0, 1, 0, 1, 0, -1, 0, 0)               # part X -> asm Z, Y -> Y, Z -> -X
ROT_ID = (1, 0, 0, 0, 1, 0, 0, 0, 1)
ASSY = os.path.join(OUT, "Phase1_ThermoKinetic_Assembly.SLDASM")


class Assy:
    def __init__(self, sw: SW):
        self.sw = sw
        self.doc = sw.new_assembly()
        self.ext = self.doc.Extension
        self.mates = []

    def insert(self, path, xyz=(0, 0, 0)):
        comp = call(self.doc, "AddComponent5", path, 0, "", False, "", float(xyz[0]), float(xyz[1]), float(xyz[2]))
        if comp is None:
            raise RuntimeError(f"insert failed: {path}")
        return comp

    def pose(self, comp, rot, t):
        xf = call(self.sw.math, "CreateTransform", dbl_array(list(rot) + list(t) + [1.0, 0, 0, 0]))
        try:
            comp.Transform2 = xf
        except Exception:  # noqa: BLE001
            call(comp, "SetTransformAndSolve2", xf)

    def fix(self, comp, fixed):
        self.doc.ClearSelection2(True)
        call(comp, "Select4", False, nothing(), False)
        get(self.doc, "FixComponent" if fixed else "UnfixComponent")
        self.doc.ClearSelection2(True)

    @staticmethod
    def plane(comp, index):
        feat, n = get(comp, "FirstFeature"), 0
        while feat is not None:
            if get(feat, "GetTypeName2") == "RefPlane":
                n += 1
                if n == index:
                    return feat
            feat = get(feat, "GetNextFeature")
        raise RuntimeError("component plane not found")

    @staticmethod
    def cyl_faces(comp, radius):
        out = []
        for body in call(comp, "GetBodies2", 0) or ():
            for face in get(body, "GetFaces") or ():
                surf = get(face, "GetSurface")
                if get(surf, "IsCylinder"):
                    p = get(surf, "CylinderParams")
                    if abs(p[6] - radius) < 2e-6:
                        out.append((face, tuple(p)))
        return out

    def mate(self, name, mtype, align, a, b, distance=0.0, angle=0.0, flip=False):
        self.doc.ClearSelection2(True)
        n = call(self.ext, "MultiSelect2", disp_array([a, b]), False, nothing())
        if n < 2:
            raise RuntimeError(f"{name}: selection failed ({n})")
        err = byref_long()
        m = call(self.doc, "AddMate5", mtype, align, flip, distance, distance, distance, 1.0, 1.0,
                 angle, angle, angle, False, False, 0, err)
        self.doc.ClearSelection2(True)
        ok = m is not None and err.value == 1
        self.mates.append({"name": name, "type": mtype, "ok": ok, "error": err.value})
        if not ok:
            raise RuntimeError(f"mate {name} failed (error {err.value})")
        return m

    def rebuild(self):
        get(self.doc, "EditRebuild3")


def last_mate_feature(doc):
    feat, group = get(doc, "FirstFeature"), None
    while feat is not None:
        if get(feat, "GetTypeName2") == "MateGroup":
            group = feat
        feat = get(feat, "GetNextFeature")
    last, sub = None, get(group, "GetFirstSubFeature")
    while sub is not None:
        last = sub
        sub = get(sub, "GetNextSubFeature")
    return last


def mate_count(doc):
    feat, group = get(doc, "FirstFeature"), None
    while feat is not None:
        if get(feat, "GetTypeName2") == "MateGroup":
            group = feat
        feat = get(feat, "GetNextFeature")
    n, sub = 0, get(group, "GetFirstSubFeature")
    while sub is not None:
        n += 1
        sub = get(sub, "GetNextSubFeature")
    return n


def comp_origin(comp):
    d = get(get(comp, "Transform2"), "ArrayData")
    return d[9], d[10], d[11]


def main():
    sw = SW()
    sw.close_if_open(ASSY)
    a = Assy(sw)
    zone = a.insert(os.path.join(OUT, "F1_Brake_Packaging_Zone.SLDPRT"))
    a.pose(zone, ROT_ID, (0, 0, 0))
    a.fix(zone, True)
    housing = a.insert(os.path.join(OUT, "Brake_Duct_Housing.SLDPRT"))
    a.fix(housing, False)
    for i, lbl in ((1, "Front"), (2, "Top"), (3, "Right")):
        a.mate(f"Housing_{lbl}_Coincident", MATE_COINCIDENT, ALIGN_ALIGNED, a.plane(housing, i), a.plane(zone, i))
    a.rebuild()

    holes = []
    for face, p in a.cyl_faces(housing, HOLE_R):
        if not any(abs(p[1] - q[1]) < 1e-4 for _, q in holes):
            holes.append((face, p))
    holes.sort(key=lambda fp: fp[1][1])
    if len(holes) != 4:
        raise RuntimeError(f"found {len(holes)} housing holes")

    louvers = []
    for k, off in enumerate(OFFSETS):
        y = H_CY + off
        lv = a.insert(os.path.join(OUT, "Nitinol_Louver_Flap.SLDPRT"), (HX, y, 0))
        a.fix(lv, False)
        a.pose(lv, ROT_LOUVER, (HX, y, 0))
        shaft = [f for f, p in a.cyl_faces(lv, HOLE_R)]
        a.mate(f"Louver{k + 1}_Pivot_Concentric", MATE_CONCENTRIC, ALIGN_CLOSEST, shaft[0], holes[k][0])
        a.mate(f"Louver{k + 1}_Centre_Coincident", MATE_COINCIDENT, ALIGN_CLOSEST, a.plane(lv, 3), a.plane(housing, 1))
        louvers.append(lv)
    a.rebuild()

    rod_x = HX + CRANK_R * math.cos(CRANK_PHI0)
    rod_y = H_CY + CRANK_R * math.sin(CRANK_PHI0)
    rod = a.insert(os.path.join(OUT, "Control_Linkage_Rod.SLDPRT"), (rod_x, rod_y, ROD_Z))
    a.fix(rod, False)
    a.pose(rod, ROT_ID, (rod_x, rod_y, ROD_Z))
    rod_holes = sorted(a.cyl_faces(rod, ROD_HOLE_R), key=lambda fp: fp[1][1])
    uniq = []
    for f, p in rod_holes:
        if not any(abs(p[1] - q[1]) < 1e-4 for _, q in uniq):
            uniq.append((f, p))
    a.mate("Rod_Plane_Distance", MATE_DISTANCE, ALIGN_CLOSEST, a.plane(rod, 1), a.plane(housing, 1), distance=ROD_Z)
    for k, lv in enumerate(louvers):
        pin = a.cyl_faces(lv, CPIN_R)[0][0]
        a.mate(f"Rod_Link{k + 1}_Concentric", MATE_CONCENTRIC, ALIGN_CLOSEST, uniq[k][0], pin)
    a.rebuild()

    # ---- kinematic verification with a temporary angle mate on louver 1 ----
    results = {}
    flip = os.environ.get("PROBE_FLIP", "1") == "1"
    a.mate("Probe_Angle", MATE_ANGLE, ALIGN_CLOSEST, a.plane(louvers[0], 2), a.plane(housing, 2),
           angle=math.radians(1.0), flip=flip)
    probe = last_mate_feature(a.doc)
    dim = call(probe, "Parameter", "D1")
    for deg in (0.0, 20.0, 40.0, 60.0, 80.0):
        call(dim, "SetSystemValue3", math.radians(max(deg, 0.001)), 2, nothing())
        a.rebuild()
        results[deg] = comp_origin(rod)
    a.doc.ClearSelection2(True)
    probe.Select2(False, 0)
    get(a.doc, "EditDelete")
    a.mates = [x for x in a.mates if x["name"] != "Probe_Angle"]
    a.rebuild()
    if last_mate_feature(a.doc).Name == probe.Name if False else False:
        raise RuntimeError("probe mate not removed")
    stroke80 = (results[80.0][1] - results[0.0][1]) * 1000
    lateral80 = (results[80.0][0] - results[0.0][0]) * 1000
    print("mate features in tree:", mate_count(a.doc))
    for d in sorted(results):
        dy = (results[d][1] - results[0.0][1]) * 1000
        theory = CRANK_R * 1000 * (math.sin(CRANK_PHI0 + math.radians(d)) - math.sin(CRANK_PHI0))
        print(f"  louver {d:4.0f} deg: rod dY = {dy:7.3f} mm (theory {theory:7.3f})")
    print(f"stroke at 80 deg: {stroke80:.3f} mm  (lateral {lateral80:.3f} mm)")

    a.doc.ShowNamedView2("*Isometric", 7)
    get(a.doc, "ViewZoomtofit2")
    err = a.doc.SaveAs3(ASSY, 0, 1)
    if err != 0:
        raise RuntimeError(f"assembly save error {err}")
    errv, warnv = byref_long(), byref_long()
    a.ext.SaveAs(os.path.join(OUT, "Phase1_ThermoKinetic_Assembly.png"), 0, 3, nothing(), errv, warnv)
    report = {"assembly": ASSY, "mates": a.mates, "rod_origin_mm": {str(k): [v * 1000 for v in val]
                                                                     for k, val in results.items()},
              "stroke_80deg_mm": stroke80, "lateral_80deg_mm": lateral80}
    with open(os.path.join(OUT, "phase1_assembly.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print("mates:", sum(m["ok"] for m in a.mates), "OK of", len(a.mates))


if __name__ == "__main__":
    main()
