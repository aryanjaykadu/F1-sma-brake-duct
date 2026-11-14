"""Phase 2j - isolated wheel with a ROTATING wheel and MOVING ground, steady then transient (Fluent).

Car frame, 300 km/h: air enters at -Z 83.33 m/s. The road moves with the air: ground = moving wall (0, 0, -V).
Wheel, cover and rotor rotate about the axle (+X through the origin) at w = V / R = 231.5 rad/s, so the contact
patch moves with the road. Duct and louvers are stationary (mounted on the upright).
Steady k-omega SST first, then transient (2nd-order implicit, dt = DT) from the steady solution; drag is written
every time step for time-averaging (first DISCARD steps dropped as wash-out in the analysis).
Usage: VARIANT=Closed|Open  [MESH_TAG=""]  STEADY_ITER=400  N_STEPS=220  ITER_PER_STEP=6  DT=1e-3
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
import sys
import time

import ansys.fluent.core as pyfluent

D = PROJECT_ROOT + r"\cfd\wheel"
VARIANT = os.environ.get("VARIANT", "Closed")
TAG = os.environ.get("MESH_TAG", "")
NAME = f"Wheel_Aero_{VARIANT}{TAG}"
OUTTAG = f"{VARIANT}{TAG}_rot"
STEADY_ITER = int(os.environ.get("STEADY_ITER", "400"))
N_STEPS = int(os.environ.get("N_STEPS", "220"))
ITER_PER_STEP = int(os.environ.get("ITER_PER_STEP", "6"))
DT = float(os.environ.get("DT", "1e-3"))
V, RHO, R_TYRE = 300 / 3.6, 1.225, 0.360
OMEGA = V / R_TYRE
GROUPS = ("wheel", "cover", "rotor", "duct", "louver")


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
out = {"variant": VARIANT, "mesh": TAG or "fine", "omega_rad_s": OMEGA, "V_m_s": V, "dt_s": DT, "n_steps": N_STEPS,
       "iter_per_step": ITER_PER_STEP, "steady_iter": STEADY_ITER}
try:
    st = s.settings
    st.file.read_mesh(file_name=os.path.join(D, NAME + ".msh.h5"))
    bc = st.setup.boundary_conditions
    bc.set_zone_type(zone_list=["ground_0"], new_type="wall")
    walls = list(bc.wall.get_object_names())
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

    # moving road
    g = bc.wall["ground_0"].momentum
    g.wall_motion = "Moving Wall"
    g.relative = False
    g.rotating = False
    try:
        g.components = True
        g.velocity_components = [0.0, 0.0, -V]
    except Exception:  # noqa: BLE001
        g.speed = V
        g.direction = [0.0, 0.0, -1.0]
    out["ground_state"] = g.get_state()
    # rotating wheel, cover, rotor
    spin = [w for w in walls if w.startswith(("wheel", "cover", "rotor"))]
    for w in spin:
        m = bc.wall[w].momentum
        m.wall_motion = "Moving Wall"
        m.relative = False
        m.rotating = True
        m.rotation_speed = OMEGA
        m.rotation_axis_origin = [0.0, 0.0, 0.0]
        m.rotation_axis_direction = [1.0, 0.0, 0.0]
    out["spin_walls"] = spin
    out["wheel_state"] = bc.wall[spin[0]].momentum.get_state()
    print("GROUND", out["ground_state"], flush=True)
    print("WHEEL", out["wheel_state"], flush=True)

    try:
        st.solution.methods.p_v_coupling.flow_scheme = "Coupled"
    except Exception as exc:  # noqa: BLE001
        print("coupled not set:", exc)
    rd = st.solution.report_definitions
    zones = {gname: [w for w in walls if w.startswith(gname)] for gname in GROUPS}
    for gname, zl in zones.items():
        rd.force.create(f"drag-{gname}")
        rd.force[f"drag-{gname}"].zones = zl
        rd.force[f"drag-{gname}"].force_vector = [0.0, 0.0, -1.0]
    rd.force.create("drag-total")
    rd.force["drag-total"].zones = [w for w in walls if not w.startswith("ground")]   # the road is not the car
    rd.force["drag-total"].force_vector = [0.0, 0.0, -1.0]
    rf = st.solution.monitor.report_files
    rf.create("drag-steady")
    rf["drag-steady"].report_defs = ["drag-total", "drag-duct", "drag-wheel"]
    rf["drag-steady"].file_name = os.path.join(D, f"drag_history_{OUTTAG}_steady.out")

    st.solution.initialization.hybrid_initialize()
    t0 = time.time()
    st.solution.run_calculation.iterate(iter_count=STEADY_ITER)
    out["steady_minutes"] = (time.time() - t0) / 60
    for gname in GROUPS + ("total",):
        out[f"steady_drag_{gname}_N"] = value(rd, f"drag-{gname}")
    out["steady_Cd"] = out["steady_drag_total_N"] / (0.5 * RHO * V ** 2 * 0.720 * 0.295)
    print("STEADY", {k: v for k, v in out.items() if k.startswith("steady")}, flush=True)
    rf["drag-steady"].active = False if hasattr(rf["drag-steady"], "active") else None

    # transient continuation
    st.setup.general.solver.time = "transient"
    tcon = st.solution.run_calculation.transient_controls
    try:
        tcon.time_step_size = DT
    except Exception as exc:  # noqa: BLE001
        print("time_step_size:", exc)
    try:
        tcon.max_iter_per_time_step = ITER_PER_STEP
    except Exception as exc:  # noqa: BLE001
        print("max_iter_per_time_step:", exc)
    rf.create("drag-transient")
    rf["drag-transient"].report_defs = ["drag-total", "drag-duct", "drag-wheel"]
    rf["drag-transient"].file_name = os.path.join(D, f"drag_history_{OUTTAG}_transient.out")
    try:
        rf["drag-transient"].frequency_of = "time-step"
    except Exception as exc:  # noqa: BLE001
        print("frequency_of:", exc)
    out["transient_report_state"] = rf["drag-transient"].get_state()
    t0 = time.time()
    st.solution.run_calculation.dual_time_iterate(time_step_count=N_STEPS, max_iter_per_step=ITER_PER_STEP)
    out["transient_minutes"] = (time.time() - t0) / 60
    for gname in GROUPS + ("total",):
        out[f"final_drag_{gname}_N"] = value(rd, f"drag-{gname}")
    cas = os.path.join(D, NAME + "_rot.cas.h5")
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
json.dump(out, open(os.path.join(D, f"aero_{OUTTAG}.json"), "w"), indent=2, default=str)
print("RESULT", json.dumps({k: v for k, v in out.items() if "state" not in k}, default=str), flush=True)
