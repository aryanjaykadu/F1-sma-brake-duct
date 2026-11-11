"""Phase 2h - isolated-wheel aero solve at 300 km/h (Fluent), louvers closed or open.

Incompressible air (rho 1.225, mu 1.7894e-5), k-omega SST, isothermal (energy off: drag only), pressure-based
coupled. Stationary wheel and slip ground (a rotating wheel on a moving ground is not modelled).
Reports drag (force along -z, the air's direction of travel) on wheel, cover, rotor, duct and louvers,
duct throat mass flow, and the drag coefficient of the assembly referenced to the wheel frontal area.
Usage: VARIANT=Closed|Open  ITER=600
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
import sys
import time

import ansys.fluent.core as pyfluent

D = PROJECT_ROOT + r"\cfd\wheel"
VARIANT = os.environ.get("VARIANT", "Closed")
ITER = int(os.environ.get("ITER", "600"))
NAME = f"Wheel_Aero_{VARIANT}"
V = 300 / 3.6
RHO = 1.225
A_REF = 0.720 * 0.295                       # wheel frontal area (OD x width) [m2]
GROUPS = ("wheel", "cover", "rotor", "duct", "louver")


def launch():
    for i in range(5):
        try:
            return pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                          dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=D)
        except Exception as exc:  # noqa: BLE001
            print("launch retry", i + 1, type(exc).__name__)
            time.sleep(45)
    sys.exit("launch failed")


def value(rd, name):
    v = rd.compute(report_defs=[name])[0][name]
    return float(v[0] if isinstance(v, (list, tuple)) else v)


s = launch()
out = {"variant": VARIANT, "iterations": ITER, "V_m_s": V, "A_ref_m2": A_REF}
try:
    st = s.settings
    st.file.read_mesh(file_name=os.path.join(D, NAME + ".msh.h5"))
    bc = st.setup.boundary_conditions
    walls = list(bc.wall.get_object_names())
    print("WALLS", walls)
    print("SYM", list(bc.symmetry.get_object_names()), "IN", list(bc.velocity_inlet.get_object_names()),
          "OUT", list(bc.pressure_outlet.get_object_names()))
    st.setup.models.energy.enabled = False
    st.setup.models.viscous.model = "k-omega"
    st.setup.models.viscous.k_omega_model = "sst"
    air = st.setup.materials.fluid["air"]
    air.density.option = "constant"
    air.density.value = RHO
    inlet = bc.velocity_inlet["inlet"]
    try:
        inlet.momentum.velocity_magnitude.value = V
    except Exception:  # noqa: BLE001
        inlet.momentum.velocity.value = V
    bc.pressure_outlet["outlet"].momentum.gauge_pressure.value = 0.0
    try:
        st.solution.methods.p_v_coupling.flow_scheme = "Coupled"
    except Exception as exc:  # noqa: BLE001
        print("coupled not set:", exc)
    rd = st.solution.report_definitions
    zones = {g: [w for w in walls if w.startswith(g)] for g in GROUPS}
    print("GROUPS", {g: len(z) for g, z in zones.items()})
    for g, zl in zones.items():
        rd.force.create(f"drag-{g}")
        rd.force[f"drag-{g}"].zones = zl
        rd.force[f"drag-{g}"].force_vector = [0.0, 0.0, -1.0]
    rd.force.create("drag-total")
    rd.force["drag-total"].zones = walls
    rd.force["drag-total"].force_vector = [0.0, 0.0, -1.0]
    try:
        st.solution.monitor.report_files.create("drag-hist")
        st.solution.monitor.report_files["drag-hist"].report_defs = ["drag-total", "drag-louver"]
        st.solution.monitor.report_files["drag-hist"].file_name = os.path.join(D, f"drag_history_{VARIANT}.out")
    except Exception as exc:  # noqa: BLE001
        print("report file not set:", exc)
    st.solution.initialization.hybrid_initialize()
    st.solution.run_calculation.iterate(iter_count=ITER)

    for g in GROUPS + ("total",):
        out[f"drag_{g}_N"] = value(rd, f"drag-{g}")
    q = 0.5 * RHO * V ** 2
    out["Cd_assembly"] = out["drag_total_N"] / (q * A_REF)
    surf = st.results.surfaces
    surf.iso_surface.create("throat_x")
    surf.iso_surface["throat_x"].field = "x-coordinate"
    surf.iso_surface["throat_x"].iso_values = [0.038]
    surf.iso_clip.create("throat_y")
    surf.iso_clip["throat_y"].field = "y-coordinate"
    surf.iso_clip["throat_y"].surfaces = ["throat_x"]
    surf.iso_clip["throat_y"].range.minimum = 0.090
    surf.iso_clip["throat_y"].range.maximum = 0.190
    surf.iso_clip.create("throat")
    surf.iso_clip["throat"].field = "z-coordinate"
    surf.iso_clip["throat"].surfaces = ["throat_y"]
    surf.iso_clip["throat"].range.minimum = -0.030
    surf.iso_clip["throat"].range.maximum = 0.030
    rd.surface.create("throat-mdot")
    rd.surface["throat-mdot"].report_type = "surface-massflowrate"
    rd.surface["throat-mdot"].surface_names = ["throat"]
    out["throat_mass_flow_kg_s"] = -value(rd, "throat-mdot")
    surf.iso_surface.create("scoop_in")
    surf.iso_surface["scoop_in"].field = "z-coordinate"
    surf.iso_surface["scoop_in"].iso_values = [0.290]
    surf.iso_clip.create("scoop_y")
    surf.iso_clip["scoop_y"].field = "y-coordinate"
    surf.iso_clip["scoop_y"].surfaces = ["scoop_in"]
    surf.iso_clip["scoop_y"].range.minimum = 0.090
    surf.iso_clip["scoop_y"].range.maximum = 0.190
    surf.iso_clip.create("scoop")
    surf.iso_clip["scoop"].field = "x-coordinate"
    surf.iso_clip["scoop"].surfaces = ["scoop_y"]
    surf.iso_clip["scoop"].range.minimum = 0.100
    surf.iso_clip["scoop"].range.maximum = 0.160
    rd.surface.create("scoop-mdot")
    rd.surface["scoop-mdot"].report_type = "surface-massflowrate"
    rd.surface["scoop-mdot"].surface_names = ["scoop"]
    out["scoop_mass_flow_kg_s"] = -value(rd, "scoop-mdot")
    out["ideal_capture_kg_s"] = RHO * V * 0.060 * 0.100
    cas = os.path.join(D, NAME + ".cas.h5")
    for f in (cas, cas.replace(".cas.h5", ".dat.h5")):
        if os.path.exists(f):
            os.remove(f)
    st.file.write(file_type="case-data", file_name=cas)
    out["status"] = "COMPLETED"
except Exception as exc:  # noqa: BLE001
    out["status"] = f"FAILED: {exc}"
    print("FAILED", exc)
finally:
    s.exit()
json.dump(out, open(os.path.join(D, f"aero_{VARIANT}.json"), "w"), indent=2)
print("RESULT", json.dumps(out))
