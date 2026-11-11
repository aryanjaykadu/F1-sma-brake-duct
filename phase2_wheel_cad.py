"""Phase 2f - isolated-wheel aerodynamic domain (SOLIDWORKS), louvers closed or open.

Wheel frame: X = axle (+X inboard), Y = up, Z = fore-aft. The car drives toward +Z, so the oncoming air
travels in -Z: inlet face at z = +1.2 m, outlet at z = -2.5 m, slip ground at y = -0.355 m (5 mm tyre
contact patch), symmetry far-field elsewhere.

Parts (subtracted from the air box; only the air is meshed, every part is a wall):
  wheel        tyre + rim drum, OD 720 mm, ID 400 mm, x[-0.190, 0.095]
  cover        closed outboard wheel cover, x[-0.200, -0.190], OD 720 mm
  rotor        328 x 32 mm disc
  duct         Phase 1 throat (x 18-98 mm) + new forward-facing inlet scoop and plenum on the inboard side
  louver1..4   2 x 18 x 56 mm, closed (flap across the throat) or open (90 deg, along the throat)
Every air-side face is named after the part it touches (e.g. louver2_3), so Fluent can split the drag.
Run with LOUVERS=closed|open.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import os

from phase2_domain_cad import rect_yz, x_extrude
from swlib import END_MID_PLANE, SW, call, disp_array, get

OUT = PROJECT_ROOT + r"\cfd\wheel"
OPEN = os.environ.get("LOUVERS", "closed").lower() == "open"
NAME = f"Wheel_Aero_{'Open' if OPEN else 'Closed'}"
BOX = {"x": (-0.8, 1.2), "y": (-0.355, 1.0), "z": (-2.5, 1.2)}
R_TYRE, R_RIM, W_X0, W_X1, COVER_X0 = 0.360, 0.200, -0.190, 0.095, -0.200
ROTOR_R, ROTOR_T = 0.164, 0.032
H_CY, H_X0, H_X1 = 0.140, 0.018, 0.098
L_X, L_T, L_W, L_L, PITCH = 0.058, 0.002, 0.018, 0.056, 0.020
SWBODYCUT, SWBODYADD = 15902, 15903

# Duct = one hollow L-shaped solid: (throat outer + scoop outer) minus (throat passage + scoop passage).
THROAT_OUTER = (H_X0, 0.100, 0.088, 0.192, -0.032, 0.032)
SCOOP_OUTER = (0.098, 0.162, 0.088, 0.192, -0.032, 0.300)
THROAT_INNER = (0.010, 0.101, 0.090, 0.190, -0.030, 0.030)     # opens the outboard end, cuts into the scoop
SCOOP_INNER = (0.100, 0.160, 0.090, 0.190, -0.030, 0.310)      # opens the forward-facing inlet at z = 0.300
DUCT_BBOX = (H_X0, 0.088, -0.032, 0.162, 0.192, 0.300)


def louver_boxes():
    out = []
    for k in range(4):
        yk = H_CY + (k - 1.5) * PITCH
        if OPEN:
            out.append((f"louver{k + 1}", L_X - L_W / 2, L_X + L_W / 2, yk - L_T / 2, yk + L_T / 2, -L_L / 2, L_L / 2))
        else:
            out.append((f"louver{k + 1}", L_X - L_T / 2, L_X + L_T / 2, yk - L_W / 2, yk + L_W / 2, -L_L / 2, L_L / 2))
    return out


def box_body(p, x0, x1, y0, y1, z0, z1, name):
    sk = p.sketch_on(p.plane(3))
    rect_yz(p, sk, y0, y1, z0, z1)
    x_extrude(p, p.close_sketch(), x0, x1, name)


def disc_body(p, r_out, r_in, x0, x1, name):
    sk = p.sketch_on(p.plane(3))
    p.circles(sk, [(0, 0, 0)], r_out)
    if r_in:
        p.circles(sk, [(0, 0, 0)], r_in)
    x_extrude(p, p.close_sketch(), x0, x1, name)


def main():
    os.makedirs(OUT, exist_ok=True)
    sw = SW()
    path = os.path.join(OUT, NAME + ".SLDPRT")
    sw.close_if_open(path)
    p = sw.new_part()
    box_body(p, *BOX["x"], *BOX["y"], *BOX["z"], "air_box")
    fluid = p.bodies()[0]
    tools = []                                     # (part, bbox) for face classification

    def add(fn, part, bbox):
        before = len(p.bodies())
        fn()
        if len(p.bodies()) != before + 1:
            raise RuntimeError(f"{part}: expected a new body")
        tools.append((part, bbox))

    add(lambda: disc_body(p, R_TYRE, R_RIM, W_X0, W_X1, "wheel"), "wheel",
        (W_X0, -R_TYRE, -R_TYRE, W_X1, R_TYRE, R_TYRE))
    WHEEL_CLASSIFY = (COVER_X0, -R_TYRE, -R_TYRE, W_X1, R_TYRE, R_TYRE)   # tyre face merges with cover rim
    add(lambda: disc_body(p, R_TYRE, None, COVER_X0, W_X0, "cover"), "cover",
        (COVER_X0, -R_TYRE, -R_TYRE, W_X0, R_TYRE, R_TYRE))
    sk = p.sketch_on(p.plane(3))
    p.circles(sk, [(0, 0, 0)], ROTOR_R)
    before = len(p.bodies())
    p.extrude(p.close_sketch(), ROTOR_T, end=END_MID_PLANE, merge=False, name="rotor")
    if len(p.bodies()) != before + 1:
        raise RuntimeError("rotor body")
    tools.append(("rotor", (-ROTOR_T / 2, -ROTOR_R, -ROTOR_R, ROTOR_T / 2, ROTOR_R, ROTOR_R)))
    def find(bbox):
        for b in p.bodies():
            bb = get(b, "GetBodyBox")
            if all(abs(bb[i] - bbox[i]) < 2e-5 for i in range(6)):
                return b
        raise RuntimeError(f"body with box {bbox} not found")

    def combine(op, main_bbox, tool_bboxes, what):
        main = find(main_bbox)
        tl = [find(t) for t in tool_bboxes]
        if op == SWBODYADD:              # add: every body (main included) goes in the tool list
            tl = [main] + tl
        p.doc.ClearSelection2(True)
        if call(p.fm, "InsertCombineFeature", op, main, disp_array(tl)) is None:
            raise RuntimeError(f"combine failed: {what}")
        p.rebuild()

    def bb(x0, x1, y0, y1, z0, z1):
        return (x0, y0, z0, x1, y1, z1)

    # ---- duct as one hollow solid ----
    for nm, b6 in (("throat_outer", THROAT_OUTER), ("scoop_outer", SCOOP_OUTER)):
        box_body(p, *b6, nm)
    combine(SWBODYADD, bb(*SCOOP_OUTER), [bb(*THROAT_OUTER)], "duct union")
    for nm, b6 in (("throat_inner", THROAT_INNER), ("scoop_inner", SCOOP_INNER)):
        box_body(p, *b6, nm)
    combine(SWBODYCUT, DUCT_BBOX, [bb(*THROAT_INNER), bb(*SCOOP_INNER)], "duct passages")
    tools.append(("duct", DUCT_BBOX))
    for part, x0, x1, y0, y1, z0, z1 in louver_boxes():
        add(lambda a=(x0, x1, y0, y1, z0, z1), n=part: box_body(p, *a, n), part, (x0, y0, z0, x1, y1, z1))

    air_bbox = (BOX["x"][0], BOX["y"][0], BOX["z"][0], BOX["x"][1], BOX["y"][1], BOX["z"][1])
    for name, tb in list(tools):
        try:
            combine(SWBODYCUT, air_bbox, [tb], f"subtract {name}")
        except RuntimeError as exc:
            raise RuntimeError(f"{exc} (tool box {[round(v, 4) for v in tb]})")
    bodies = p.bodies()
    if len(bodies) != 1:
        raise RuntimeError(f"expected one air body, got {len(bodies)}")
    air = bodies[0]
    air.Name = "fluid_domain"

    # ---- name every air-side face by the part it touches ----
    tol = 2e-5
    priority = [t for t in tools if t[0].startswith("louver")] + [t for t in tools if t[0] == "rotor"] + \
               [t for t in tools if t[0] == "duct"] + [t for t in tools if t[0] == "cover"] + \
               [("wheel", WHEEL_CLASSIFY)]
    counts, labels = {}, {}
    for face in get(air, "GetFaces"):
        b = get(face, "GetBox")
        x0, y0, z0, x1, y1, z1 = b
        part = None
        if abs(z0 - BOX["z"][1]) < tol and abs(z1 - BOX["z"][1]) < tol:
            part = "inlet"
        elif abs(z0 - BOX["z"][0]) < tol and abs(z1 - BOX["z"][0]) < tol:
            part = "outlet"
        elif abs(y0 - BOX["y"][0]) < tol and abs(y1 - BOX["y"][0]) < tol:
            part = "ground"
        elif (abs(y0 - y1) < tol and abs(y0 - BOX["y"][1]) < tol) or (abs(x0 - x1) < tol and
                                                                          (abs(x0 - BOX["x"][0]) < tol or abs(x0 - BOX["x"][1]) < tol)):
            part = "farfield"
        else:
            for name, tb in priority:
                if all(b[i] >= tb[i] - tol for i in range(3)) and all(b[i + 3] <= tb[i + 3] + tol for i in range(3)):
                    part = "duct" if name == "duct" else name
                    break
        if part is None:
            raise RuntimeError(f"unclassified face box {[round(v, 4) for v in b]}")
        i = counts.get(part, 0)
        counts[part] = i + 1
        label = part if part in ("inlet", "outlet") else f"{part}_{i}"
        if not call(p.doc, "SetEntityName", face, label):
            raise RuntimeError(f"could not name face {label}")
        labels.setdefault(part, []).append(label)
    print("FACES", {k: len(v) for k, v in labels.items()})
    p.view_iso()
    p.save(path)
    p.export(os.path.join(OUT, NAME + ".x_t"))
    p.export(os.path.join(OUT, NAME + ".png"))
    json.dump({"open": OPEN, "box_m": BOX, "labels": labels}, open(os.path.join(OUT, NAME + ".json"), "w"), indent=1)
    print("SAVED", path)


if __name__ == "__main__":
    main()
