"""EV re-target - half-car Fluent solve with a porous radiator, moving road, shutters open or closed.

Steady k-omega SST, incompressible air. Inlet V (+X), road = moving wall at V, symmetry on z = 0 and the far field.
Radiator = porous zone: inertial resistance C2 = K / t = 10 / 0.025 m = 400 1/m along X, 1000x larger across.
Car drag (full car, x2) = wall forces on every car wall (incl. the sealed pocket) + force on the radiator core
(p_front - p_back) x A. Radiator mass flow from the core's back face (x2).
Usage: SHUTTER=open|closed  SPEED_KMH=100  ITER=900
"""
import json
import os
import sys
import time

import ansys.fluent.core as pyfluent

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfd")
OPEN = os.environ.get("SHUTTER", "open").lower() == "open"
VAR = "Open" if OPEN else "Closed"
SPEED = float(os.environ.get("SPEED_KMH", "100"))
ITER = int(os.environ.get("ITER", "900"))
V, RHO = SPEED / 3.6, 1.225
A_RAD_HALF = 0.35 * 0.225
C2 = 10.0 / 0.025
TAG = f"{VAR}_{SPEED:.0f}"


def launch():
    for i in range(5):
        try:
            return pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                          dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=D)
        except Exception as exc:  # noqa: BLE001
            print("launch retry", i + 1, type(exc).__name__, flush=True)
            time.sleep(60)
    sys.exit("launch failed")


def value(rd, name):
    v = rd.compute(report_defs=[name])[0][name]
    return float(v[0] if isinstance(v, (list, tuple)) else v)


s = launch()
out = {"variant": VAR, "speed_kmh": SPEED, "iter": ITER}
try:
    st = s.settings
    st.file.read_mesh(file_name=os.path.join(D, f"EV_Front_{VAR}.msh.h5"))
    bc = st.setup.boundary_conditions
    # The mesher typed the core's front/back faces as Fluent "radiator" boundaries (inferred from the label).
    # The core is modelled as a porous zone instead, so those faces must be plain interior faces.
    rad_bc = list(bc.radiator.get_object_names()) if "radiator" in bc.child_names else []
    if rad_bc:
        bc.set_zone_type(zone_list=rad_bc, new_type="interior")
    # region-to-region faces from shared topology (e.g. the pocket-to-core face when the shutter is closed)
    interior_rr = [z for z in (bc.interior.get_object_names() if "interior" in bc.child_names else [])
                   if z.startswith("region") and "-region" in z]
    interfaces = rad_bc + interior_rr
    st.solution.initialization.hybrid_initialize()          # needed before geometric report values
    rd0 = st.solution.report_definitions

    def centroid(z, axis):
        nm = f"c{axis}-{z}"[:40]
        rd0.surface.create(nm)
        rd0.surface[nm].report_type = "surface-areaavg"
        rd0.surface[nm].field = f"{axis}-coordinate"
        rd0.surface[nm].surface_names = [z]
        return value(rd0, nm)
    # the radiator wall lying on the symmetry plane (z = 0) must be a symmetry face, not a wall
    rwalls = [w for w in bc.wall.get_object_names()
              if w.startswith(("id_radiator", "radiator", "id_plenum", "plenum")) or w == "wall"]
    on_sym = [w for w in rwalls if abs(centroid(w, "z")) < 1e-4]
    if on_sym:
        bc.set_zone_type(zone_list=on_sym, new_type="symmetry")
    out["interfaces"] = {z: centroid(z, "x") for z in interfaces}
    out["radiator_walls_to_symmetry"] = on_sym
    walls = list(bc.wall.get_object_names())
    print("INTERFACES", out["interfaces"], "SYM", on_sym, "\nWALLS", walls, flush=True)
    st.setup.models.energy.enabled = False
    st.setup.models.viscous.model = "k-omega"
    st.setup.models.viscous.k_omega_model = "sst"
    air = st.setup.materials.fluid["air"]
    air.density.option = "constant"
    air.density.value = RHO
    try:
        bc.velocity_inlet["inlet"].momentum.velocity_magnitude.value = V
    except Exception:  # noqa: BLE001
        bc.velocity_inlet["inlet"].momentum.velocity.value = V
    bc.pressure_outlet["outlet"].momentum.gauge_pressure.value = 0.0
    ground = [w for w in walls if w.startswith("ground")]
    for gname in ground:
        g = bc.wall[gname].momentum
        g.wall_motion = "Moving Wall"
        g.relative = False
        g.rotating = False
        g.components = True
        g.velocity_components = [V, 0.0, 0.0]
    # porous radiator
    cz = st.setup.cell_zone_conditions.fluid
    zones = list(cz.get_object_names())
    print("CELL ZONES", zones, flush=True)
    rz = cz["radiator"]
    pz = rz.porous_zone
    print("POROUS children", list(pz.child_names), flush=True)
    pz.porous = True
    for nm, val in (("direction_1_vector", [1, 0, 0]), ("direction_2_vector", [0, 1, 0])):
        try:
            setattr(pz, nm, val)
        except Exception as exc:  # noqa: BLE001
            print("set", nm, exc, flush=True)
    for nm, val in (("viscous_resistance", [0.0, 0.0, 0.0]), ("inertial_resistance", [C2, 1000 * C2, 1000 * C2])):
        try:
            setattr(pz, nm, val)
        except Exception as exc:  # noqa: BLE001
            print("set", nm, exc, flush=True)
    out["porous_state"] = pz.get_state()
    print("POROUS", out["porous_state"], flush=True)
    try:
        st.solution.methods.p_v_coupling.flow_scheme = "Coupled"
    except Exception as exc:  # noqa: BLE001
        print("coupled:", exc)

    rd = st.solution.report_definitions
    car_walls = [w for w in walls if not w.startswith("ground")]
    rd.force.create("drag-walls")
    rd.force["drag-walls"].zones = car_walls
    rd.force["drag-walls"].force_vector = [1.0, 0.0, 0.0]
    # measurement planes 0.5 mm in front of and behind the 25 mm core, clipped to the grille duct
    surf = st.results.surfaces
    planes = {}
    for key, xpos in (("front", 0.1995), ("back", 0.2255)):
        surf.iso_surface.create(f"{key}_x")
        surf.iso_surface[f"{key}_x"].field = "x-coordinate"
        surf.iso_surface[f"{key}_x"].iso_values = [xpos]
        surf.iso_clip.create(f"{key}_y")
        surf.iso_clip[f"{key}_y"].field = "y-coordinate"
        surf.iso_clip[f"{key}_y"].surfaces = [f"{key}_x"]
        surf.iso_clip[f"{key}_y"].range.minimum = 0.471
        surf.iso_clip[f"{key}_y"].range.maximum = 0.819
        surf.iso_clip.create(f"core_{key}")
        surf.iso_clip[f"core_{key}"].field = "z-coordinate"
        surf.iso_clip[f"core_{key}"].surfaces = [f"{key}_y"]
        surf.iso_clip[f"core_{key}"].range.minimum = 0.0
        surf.iso_clip[f"core_{key}"].range.maximum = 0.2249
        for nm, typ, field in ((f"p-{key}", "surface-areaavg", "pressure"), (f"a-{key}", "surface-area", None),
                               (f"m-{key}", "surface-massflowrate", None)):
            rd.surface.create(nm)
            rd.surface[nm].report_type = typ
            if field:
                rd.surface[nm].field = field
            rd.surface[nm].surface_names = [f"core_{key}"]
        planes[key] = f"core_{key}"
    rf = st.solution.monitor.report_files
    rf.create("drag-hist")
    rf["drag-hist"].report_defs = ["drag-walls"]
    rf["drag-hist"].file_name = os.path.join(D, f"drag_history_{TAG}.out")
    st.solution.initialization.hybrid_initialize()
    st.solution.run_calculation.iterate(iter_count=ITER)

    meas = {k: {"p": value(rd, f"p-{k}"), "area": value(rd, f"a-{k}"), "m": value(rd, f"m-{k}")} for k in planes}
    out["core_planes"] = meas
    for k, mv in meas.items():
        if not 0.9 * A_RAD_HALF < mv["area"] < 1.1 * A_RAD_HALF:
            raise RuntimeError(f"{k} plane area {mv['area']:.5f} m2, expected {A_RAD_HALF:.5f}")
    f_rad = (meas["front"]["p"] - meas["back"]["p"]) * A_RAD_HALF
    faces = {"back": meas["back"]}
    back = "back"
    out["drag_walls_half_N"] = value(rd, "drag-walls")
    out["radiator_force_half_N"] = f_rad
    out["drag_total_N"] = 2 * (out["drag_walls_half_N"] + f_rad)
    out["radiator_mdot_kg_s"] = 2 * abs(faces[back]["m"])
    out["Cd"] = out["drag_total_N"] / (0.5 * RHO * V ** 2 * 2.30)
    cas = os.path.join(D, f"EV_{TAG}.cas.h5")
    for f in (cas, cas.replace(".cas.h5", ".dat.h5")):
        if os.path.exists(f):
            os.remove(f)
    st.file.write(file_type="case-data", file_name=cas)
    out["status"] = "COMPLETED"
except Exception as exc:  # noqa: BLE001
    out["status"] = f"FAILED: {exc}"
    print("FAILED", exc, flush=True)
finally:
    s.exit()
json.dump(out, open(os.path.join(D, f"ev_aero_{TAG}.json"), "w"), indent=2, default=str)
print("RESULT", json.dumps({k: v for k, v in out.items() if k != "porous_state"}, default=str), flush=True)
