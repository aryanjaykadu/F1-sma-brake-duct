"""Phase 6a - bench test rig for the SMA brake duct (SOLIDWORKS).

Builds the rig parts in the Phase 1 assembly frame (mm below; X = throat axis, air flows toward -X,
Y = up, Z = louver shaft axis, the linkage rod and spring pack are on the +Z side) and assembles them
around the Phase 1 duct housing, louvers and rod. All rig parts are modelled in place, so every
component is inserted with an identity transform and fixed.

  base plate        Al 6061, 10 mm
  duct stands x2    hold the housing at y = 88 mm
  fan plenum        3D printed, 92 mm fan to the 100 x 60 mm throat
  fan               92 x 25 mm 4-pin PWM fan (envelope)
  spring pack       six 0.5 mm NiTiCu springs (envelopes) between a crosshead on the rod and an anchor plate
  load cell         80 mm bar cell, 5 kg, carried on a post; anchor plate hangs from its free end
  bias spring       below the rod, to a bracket on the base
  IR heater         118 mm R7s 500 W halogen lamp in a U reflector, axis 23 mm from the spring line (rotor stand-in)
  blower            50 mm PWM radial blower + nozzle on the spring pack (car-speed cooling stand-in)
  heat shield       keeps the lamp off the housing and crank pins
  angle sensor      AS5600 board under a diametric magnet on louver 2's free shaft end (-Z side)
  thermocouples     TC1 on the spring pack, TC2 in the plenum air, TC3 on the reflector
  electronics box   Arduino Uno + sensor shield
Writes cad/rig/*.SLDPRT, Bench_Rig.SLDASM, STEP, renders and rig_layout.json.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import os

from phase2_domain_cad import rect_yz, x_extrude
from swlib import START_OFFSET, SW, byref_long, call, dbl_array, disp_array, get, nothing

ROOT = PROJECT_ROOT
OUT = os.path.join(ROOT, "cad", "rig")
P1 = os.path.join(ROOT, "cad")
SWBODYCUT = 15902
MM = 1e-3

ROD_X, ROD_Z = 66.34, 36.5          # rod centreline (open pose) and spring line, mm
SPRING_X = [ROD_X - 10 + 4 * k for k in range(6)]


def mm(*v):
    return [a * MM for a in v]


# ---------------------------------------------------------------- geometry helpers
def sketch_plane(p, axis):
    return p.plane({"x": 3, "y": 2, "z": 1}[axis])


def along(p, sk_feat, axis, a0, a1, name):
    """Extrude a sketch on the plane normal to `axis` so the body spans a0..a1 (m) along that axis."""
    before = {id(b) for b in p.bodies()}
    if axis == "x":
        x_extrude(p, sk_feat, a0, a1, name)
    else:
        sign = 1.0 if axis == "z" else 1.0
        if a0 < 0:
            p.extrude(sk_feat, a1 - a0, merge=False, start=START_OFFSET, start_offset=-a0 * sign, flip_start=True,
                      name=name)
        else:
            p.extrude(sk_feat, a1 - a0, merge=False, start=START_OFFSET, start_offset=a0 * sign, name=name)
    i = {"x": 0, "y": 1, "z": 2}[axis]
    for b in p.bodies():
        bb = get(b, "GetBodyBox")
        if abs(bb[i] - a0) < 1e-6 and abs(bb[i + 3] - a1) < 1e-6:
            return b
    boxes = [[round(v / MM, 2) for v in get(b, "GetBodyBox")] for b in p.bodies()]
    raise RuntimeError(f"{name}: no body spans {axis} {a0 / MM:.2f}..{a1 / MM:.2f} mm; bodies {boxes}")


def box(p, x0, x1, y0, y1, z0, z1, name):
    x0, x1, y0, y1, z0, z1 = mm(x0, x1, y0, y1, z0, z1)
    sk = p.sketch_on(sketch_plane(p, "x"))
    rect_yz(p, sk, y0, y1, z0, z1)
    return along(p, p.close_sketch(), "x", x0, x1, name)


def cyl(p, axis, c, r, a0, a1, name):
    """Cylinder of radius r along `axis` through centre c = (u, v) in the other two coordinates (mm)."""
    u, v = mm(*c)
    sk = p.sketch_on(sketch_plane(p, axis))
    pt = {"x": (0, u, v), "y": (u, 0, v), "z": (u, v, 0)}[axis]
    p.circles(sk, [pt], r * MM)
    return along(p, p.close_sketch(), axis, a0 * MM, a1 * MM, name)


def subtract(p, main, tools, what):
    p.doc.ClearSelection2(True)
    if call(p.fm, "InsertCombineFeature", SWBODYCUT, main, disp_array(tools)) is None:
        raise RuntimeError(f"cut failed: {what}")
    p.rebuild()


def colour(p, rgb):
    p.doc.MaterialPropertyValues = dbl_array(list(rgb) + [1.0, 1.0, 0.5, 0.4, 0.0, 0.0])


# ---------------------------------------------------------------- parts
def base_plate(p):
    b = box(p, -40, 265, -10, 0, -110, 135, "plate")
    for i, (x, z) in enumerate([(-30, -100), (-30, 125), (255, -100), (255, 125)]):
        h = cyl(p, "y", (x, z), 3.3, -12, 2, f"h{i}")
        plate = [q for q in p.bodies() if abs(get(q, "GetBodyBox")[3] - get(q, "GetBodyBox")[0]) > 0.2][0]
        subtract(p, plate, [h], f"hole {i}")
    return (0.75, 0.76, 0.78)


def duct_stands(p):
    for i, (x0, x1) in enumerate(((24, 34), (82, 92))):
        box(p, x0, x1, 0, 88, -32, 32, f"stand{i}")
    return (0.2, 0.2, 0.22)


def plenum(p):
    outer = box(p, 98, 158, 80, 200, -50, 50, "outer")
    inner = box(p, 101, 155, 83, 197, -47, 47, "inner")
    front = box(p, 97, 102, 90, 190, -30, 30, "throat_port")
    rear = cyl(p, "x", (140, 0), 44, 154, 159, "fan_port")
    tc = cyl(p, "y", (128, 0), 1.0, 196, 201, "tc_port")
    subtract(p, outer, [inner, front, rear, tc], "plenum")
    box(p, 118, 138, 0, 80, -40, 40, "plenum_stand")
    return (0.95, 0.45, 0.1)


def fan(p):
    frame = box(p, 158, 183, 94, 186, -46, 46, "frame")
    bore = cyl(p, "x", (140, 0), 45, 157, 184, "bore")
    subtract(p, frame, [bore], "fan bore")
    cyl(p, "x", (140, 0), 19, 160, 181, "hub")
    box(p, 160, 181, 0, 94, -40, 40, "fan_stand")
    # 50 mm PWM radial blower on the plenum lid, nozzle 2.4 mm from the spring pack (~13 m/s at 100 %)
    box(p, 95, 110, 200, 250, 25, 50, "blower_50mm")
    box(p, 80, 95, 214, 228, 31, 42, "blower_nozzle")
    return (0.1, 0.1, 0.1)


def spring_pack(p):
    box(p, 54, 79, 190, 196, 34, 39, "crosshead")
    for k, x in enumerate(SPRING_X):
        cyl(p, "y", (x, ROD_Z), 1.25, 196, 246, f"sma_spring{k + 1}")
    box(p, 54, 79, 246, 250, 34, 39, "anchor_plate")
    cyl(p, "y", (ROD_X, ROD_Z), 1.5, 250, 262, "hook")
    return (0.35, 0.35, 0.38)


def load_cell(p):
    box(p, -10, 70, 262, 274.7, 30.15, 42.85, "bar_cell")
    box(p, -10, 6, 0, 262, 28, 45, "post")
    return (0.7, 0.72, 0.75)


def bias_spring(p):
    cyl(p, "y", (ROD_X, ROD_Z), 2.0, 54, 104, "bias_spring")
    box(p, 56, 77, 0, 54, 31, 42, "bias_bracket")
    return (0.55, 0.55, 0.6)


def heater(p):
    # lamp axis 23 mm from the spring line: ~143 kW/m2 at 100 % (line source, 110 kW/m2 at 30 mm)
    refl = box(p, 44, 89, 155, 283, 51, 77, "reflector")
    cavity = box(p, 47, 86, 158, 280, 45, 74, "cavity")
    subtract(p, refl, [cavity], "reflector")
    cyl(p, "y", (ROD_X, 59.5), 5.0, 160, 278, "r7s_lamp")
    box(p, 58, 75, 0, 155, 55, 73, "heater_column")
    box(p, 30, 96, 150, 192, 46, 48, "heat_shield")
    return (0.85, 0.75, 0.2)


def angle_sensor(p):
    cyl(p, "z", (58, 130), 3.0, -35.5, -33.0, "magnet")
    box(p, 46.5, 69.5, 118.5, 141.5, -39.1, -37.5, "as5600_board")
    box(p, 40, 46.5, 118.5, 141.5, -39.1, -32, "sensor_bracket")
    return (0.1, 0.45, 0.2)


def thermocouples(p):
    cyl(p, "z", (ROD_X, 221), 0.75, 38, 50, "tc1_spring")
    cyl(p, "y", (128, 0), 0.75, 190, 215, "tc2_air")
    cyl(p, "x", (219, 64), 0.75, 89, 104, "tc3_reflector")
    return (0.8, 0.1, 0.1)


def electronics(p):
    shell = box(p, 165, 250, 0, 45, 45, 115, "enclosure")
    inner = box(p, 167.5, 247.5, 2.5, 50, 47.5, 112.5, "cavity")
    subtract(p, shell, [inner], "enclosure")
    box(p, 173, 241.6, 6, 7.6, 52, 105.4, "arduino_uno")
    box(p, 173, 241.6, 20, 21.6, 52, 105.4, "sensor_shield")
    return (0.15, 0.3, 0.6)


PARTS = [("Rig_Base_Plate", base_plate), ("Rig_Duct_Stands", duct_stands), ("Rig_Fan_Plenum", plenum),
         ("Rig_Fan_92mm", fan), ("Rig_SMA_Spring_Pack", spring_pack), ("Rig_Load_Cell", load_cell),
         ("Rig_Bias_Spring", bias_spring), ("Rig_IR_Heater", heater), ("Rig_Angle_Sensor", angle_sensor),
         ("Rig_Thermocouples", thermocouples), ("Rig_Electronics_Box", electronics)]

# Phase 1 parts with their open (80 deg) pose; the rod moves to the crank-pin side (+Z).
ROT_ID = (1, 0, 0, 0, 1, 0, 0, 0, 1)
P1_PARTS = [("Brake_Duct_Housing", ROT_ID, (0, 0, 0))] + \
    [("Nitinol_Louver_Flap", (0.0, 0.0, 1.0, -0.98481, 0.17365, 0.0, -0.17365, -0.98481, 0.0), (58, y, 0))
     for y in (110, 130, 150, 170)] + [("Control_Linkage_Rod", ROT_ID, (ROD_X, 147.0, 35.5))]


def build_part(sw, name, fn):
    path = os.path.join(OUT, name + ".SLDPRT")
    sw.close_if_open(path)
    p = sw.new_part()
    rgb = fn(p)
    colour(p, rgb)
    p.rebuild()
    p.view_iso()
    p.save(path)
    p.export(os.path.join(OUT, name + ".STEP"))
    boxes = {get(b, "Name"): [round(v / MM, 2) for v in get(b, "GetBodyBox")] for b in p.bodies()}
    sw.app.CloseDoc(get(p.doc, "GetTitle"))
    print("PART", name, len(boxes), "bodies")
    return path, boxes


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    sw.app.CloseAllDocuments(True)
    layout = {"parts": {}}
    paths = []
    for name, fn in PARTS:
        path, boxes = build_part(sw, name, fn)
        layout["parts"][name] = boxes
        paths.append((path, ROT_ID, (0, 0, 0)))
    for name, rot, t in P1_PARTS:
        paths.append((os.path.join(P1, name + ".SLDPRT"), rot, t))

    asm_path = os.path.join(OUT, "Bench_Rig.SLDASM")
    sw.close_if_open(asm_path)
    doc = sw.new_assembly()
    for path, rot, t in paths:
        err, warn = byref_long(), byref_long()
        sw.app.OpenDoc6(path, 1, 1, "", err, warn)          # load the part so the insert is instant
        sw.app.ActivateDoc3(get(doc, "GetTitle"), False, 0, byref_long())
        comp = call(doc, "AddComponent5", path, 0, "", False, "", 0.0, 0.0, 0.0)
        if comp is None:
            raise RuntimeError(f"insert failed {path}")
        xf = call(sw.math, "CreateTransform", dbl_array(list(rot) + [v * MM for v in t] + [1.0, 0, 0, 0]))
        comp.Transform2 = xf
        doc.ClearSelection2(True)
        call(comp, "Select4", False, nothing(), False)
        get(doc, "FixComponent")
        doc.ClearSelection2(True)
    doc.ForceRebuild3(False)
    for title in [os.path.basename(p) for p, _, _ in paths]:
        try:
            sw.app.CloseDoc(title)
        except Exception:  # noqa: BLE001
            pass
    sw.app.ActivateDoc3(get(doc, "GetTitle"), False, 0, byref_long())

    # interference check
    clashes = []
    try:
        idm = get(doc, "InterferenceDetectionManager")
        idm.TreatCoincidenceAsInterference = False
        idm.IncludeMultibodyPartInterferences = False
        res = get(idm, "GetInterferences") or ()
        for it in res:
            comps = [get(c, "Name2") for c in (get(it, "Components") or ())]
            clashes.append({"components": comps, "volume_mm3": round(get(it, "Volume") * 1e9, 1)})
        idm.Done()
    except Exception as exc:  # noqa: BLE001
        clashes = [f"interference check failed: {exc}"]
    layout["interferences"] = clashes
    print("INTERFERENCES", json.dumps(clashes, indent=1))

    ext = doc.Extension
    err, warn = byref_long(), byref_long()
    doc.ShowNamedView2("*Isometric", 7)
    get(doc, "ViewZoomtofit2")
    if not ext.SaveAs(asm_path, 0, 1, nothing(), err, warn):
        raise RuntimeError(f"assembly save failed {err.value}")
    for view, fname in (("*Isometric", "Bench_Rig_iso.png"), ("*Front", "Bench_Rig_front.png"),
                        ("*Top", "Bench_Rig_top.png"), ("*Trimetric", "Bench_Rig_trimetric.png")):
        doc.ShowNamedView2(view, -1)
        get(doc, "ViewZoomtofit2")
        doc.GraphicsRedraw2()
        ext.SaveAs(os.path.join(OUT, fname), 0, 1 | 2, nothing(), err, warn)
    ext.SaveAs(os.path.join(OUT, "Bench_Rig.STEP"), 0, 1 | 2, nothing(), err, warn)
    layout["assembly"] = asm_path
    json.dump(layout, open(os.path.join(OUT, "rig_layout.json"), "w"), indent=1)
    print("SAVED", asm_path)


if __name__ == "__main__":
    main()
