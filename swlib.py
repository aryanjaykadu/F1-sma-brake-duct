"""Thin, tested helpers around the SOLIDWORKS COM API (late bound, pywin32)."""
from __future__ import annotations

import math
import os
import types

import pythoncom
import win32com.client
import win32com.client.dynamic
from win32com.client import VARIANT

PART_TEMPLATE = r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2025\templates\MBD\part 0051mm to 0250mm.prtdot"
ASSY_TEMPLATE = r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2025\templates\MBD\assembly 0251mm to 1000mm.asmdot"

END_BLIND, END_THROUGH_ALL, END_MID_PLANE = 0, 1, 6
START_SKETCH_PLANE, START_OFFSET = 0, 3


def nothing():
    return VARIANT(pythoncom.VT_DISPATCH, None)


def dbl_array(values):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(v) for v in values])


def disp_array(objs):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, list(objs))


def byref_long():
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


def get(obj, name):
    member = getattr(obj, name)
    return member() if isinstance(member, types.MethodType) else member


def call(obj, name, *args):
    """Invoke a COM method with arguments, forcing method semantics under late binding."""
    try:
        obj._FlagAsMethod(name)
    except AttributeError:
        pass
    return getattr(obj, name)(*args)


class SW:
    def __init__(self):
        raw = win32com.client.Dispatch("SldWorks.Application")
        self.app = win32com.client.dynamic.Dispatch(raw._oleobj_)
        self.app.Visible = True
        self.math = get(self.app, "GetMathUtility")

    def close_if_open(self, path):
        doc = self.app.GetOpenDocumentByName(path)
        if doc is not None:
            self.app.CloseDoc(get(doc, "GetTitle"))

    def new_part(self):
        doc = self.app.NewDocument(PART_TEMPLATE, 0, 0.0, 0.0)
        if doc is None:
            raise RuntimeError("NewDocument(part) failed")
        return Part(self, doc)

    def new_assembly(self):
        doc = self.app.NewDocument(ASSY_TEMPLATE, 0, 0.0, 0.0)
        if doc is None:
            raise RuntimeError("NewDocument(assembly) failed")
        return doc


class Part:
    def __init__(self, sw: SW, doc):
        self.sw = sw
        self.doc = doc
        self.ext = doc.Extension
        self.sm = doc.SketchManager
        self.fm = doc.FeatureManager

    # ---------- tree ----------
    def features(self):
        feat = get(self.doc, "FirstFeature")
        while feat is not None:
            yield feat
            feat = get(feat, "GetNextFeature")

    def plane(self, index):
        n = 0
        for feat in self.features():
            if get(feat, "GetTypeName2") == "RefPlane":
                n += 1
                if n == index:
                    return feat
        raise RuntimeError(f"plane {index} not found")

    def last_feature(self, type_name):
        last = None
        for feat in self.features():
            if get(feat, "GetTypeName2") == type_name:
                last = feat
        return last

    # ---------- sketching ----------
    def sketch_on(self, plane_feat):
        self.doc.ClearSelection2(True)
        if not plane_feat.Select2(False, 0):
            raise RuntimeError("plane select failed")
        self.sm.InsertSketch(True)
        sk = get(self.sm, "ActiveSketch")
        if sk is None:
            raise RuntimeError("sketch did not open")
        return sk

    def face_at(self, x, y, z, tol=1e-6):
        best, best_d = None, None
        for body in self.bodies():
            for face in get(body, "GetFaces") or ():
                pt = call(face, "GetClosestPointOn", x, y, z)
                if not pt:
                    continue
                d = math.dist((x, y, z), (pt[0], pt[1], pt[2]))
                if best_d is None or d < best_d:
                    best, best_d = face, d
        if best is None or best_d > tol:
            raise RuntimeError(f"no face at {x, y, z} (closest {best_d})")
        return best

    def select_objects(self, objs, append=False):
        n = call(self.ext, "MultiSelect2", disp_array(objs), append, nothing())
        if n < len(objs):
            raise RuntimeError(f"MultiSelect2 selected {n}/{len(objs)}")

    def select_face_at(self, x, y, z, append=False):
        self.select_objects([self.face_at(x, y, z)], append)

    def sketch_on_face(self, x, y, z):
        self.doc.ClearSelection2(True)
        self.select_face_at(x, y, z)
        self.sm.InsertSketch(True)
        sk = get(self.sm, "ActiveSketch")
        if sk is None:
            raise RuntimeError("sketch on face did not open")
        return sk

    def to_sketch(self, sk, x, y, z):
        xf = get(sk, "ModelToSketchTransform")
        pt = call(self.sw.math, "CreatePoint", dbl_array((x, y, z)))
        local = call(pt, "MultiplyTransform", xf)
        d = get(local, "ArrayData")
        return float(d[0]), float(d[1])

    def circles(self, sk, centers, radius):
        self.sm.AddToDB = True
        try:
            for c in centers:
                sx, sy = self.to_sketch(sk, *c)
                if self.sm.CreateCircleByRadius(sx, sy, 0.0, radius) is None:
                    raise RuntimeError(f"circle failed at {c}")
        finally:
            self.sm.AddToDB = False

    def center_rect(self, sk, center, corner):
        cx, cy = self.to_sketch(sk, *center)
        kx, ky = self.to_sketch(sk, *corner)
        self.sm.AddToDB = True
        try:
            segs = self.sm.CreateCenterRectangle(cx, cy, 0.0, kx, ky, 0.0)
            if not segs:
                raise RuntimeError("center rectangle failed")
        finally:
            self.sm.AddToDB = False

    def close_sketch(self):
        self.sm.InsertSketch(True)
        feat = self.last_feature("ProfileFeature")
        if feat is None:
            raise RuntimeError("sketch feature not found")
        return feat

    # ---------- features ----------
    def extrude(self, sketch_feat, depth, end=END_BLIND, merge=True, flip=False, start=START_SKETCH_PLANE,
                start_offset=0.0, preselected=False, name=None, flip_start=False):
        if not preselected:
            self.doc.ClearSelection2(True)
            sketch_feat.Select2(False, 0)
        feat = self.fm.FeatureExtrusion3(True, flip, False, end, 0, depth, 0.0, False, False, False, False,
                                         0.0, 0.0, False, False, False, False, merge, True, True,
                                         start, start_offset, flip_start)
        if feat is None:
            raise RuntimeError(f"extrusion failed ({name})")
        if name:
            feat.Name = name
        self.doc.ClearSelection2(True)
        return feat

    def cut_through_all(self, sketch_feat, name=None):
        self.doc.ClearSelection2(True)
        sketch_feat.Select2(False, 0)
        feat = self.fm.FeatureCut4(False, False, False, END_THROUGH_ALL, END_THROUGH_ALL, 0.0, 0.0,
                                   False, False, False, False, 0.0, 0.0, False, False, False, False,
                                   False, True, True, False, False, False, START_SKETCH_PLANE, 0.0, False, False)
        if feat is None:
            raise RuntimeError(f"cut failed ({name})")
        if name:
            feat.Name = name
        self.doc.ClearSelection2(True)
        return feat

    def boss_on_face(self, pick, center, radius, length, name):
        sk = self.sketch_on_face(*pick)
        self.circles(sk, [center], radius)
        feat = self.extrude(self.close_sketch(), length, name=name)
        return feat

    def bodies(self):
        b = self.doc.GetBodies2(0, False)
        return list(b) if b else []

    def rebuild(self):
        self.doc.ForceRebuild3(False)

    def view_iso(self):
        self.doc.ShowNamedView2("*Isometric", 7)
        get(self.doc, "ViewZoomtofit2")

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.sw.close_if_open(path) if False else None
        err = self.doc.SaveAs3(path, 0, 1)
        if err != 0 or not os.path.isfile(path):
            raise RuntimeError(f"SaveAs3 {path} -> error {err}")

    def export(self, path):
        """Export copy (STEP / Parasolid / PNG) based on extension, keeping the part document open."""
        err, warn = byref_long(), byref_long()
        ok = self.ext.SaveAs(path, 0, 1 | 2, nothing(), err, warn)   # silent | copy
        if not ok or not os.path.isfile(path):
            raise RuntimeError(f"export {path} failed (err {err.value}, warn {warn.value})")
