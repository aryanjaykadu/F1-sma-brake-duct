"""Phase 2e - aerodynamic and cooling comparison of louvers open vs closed at 300 km/h.

For each saved case (CaseB = open, CaseB_closed = closed; same 300 km/h inlet and 673 K rotor) this
reports from Fluent:
  * streamwise force (along -x, the inlet-to-outlet flow direction) on the louvers, duct walls and rotor,
  * air mass flow through the duct throat (cross-section at x = 38 mm, clipped to the 100 x 60 mm opening),
  * heat removed from the rotor.
Flow enters at x = +160 mm and leaves at x = -150 mm, so drag acts in -x.
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
import sys
import time

import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
CASES = {"open": "Phase2_CaseB.cas.h5", "closed": "Phase2_CaseB_closed.cas.h5"}
THROAT_X = 0.038
Y0, Y1, Z0, Z1 = 0.090, 0.190, -0.030, 0.030


def launch():
    for i in range(5):
        try:
            return pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                          dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=CFD)
        except Exception as exc:  # noqa: BLE001
            print("launch retry", i + 1, type(exc).__name__)
            time.sleep(45)
    sys.exit("launch failed")


def value(rd, name):
    v = rd.compute(report_defs=[name])[0][name]
    return float(v[0] if isinstance(v, (list, tuple)) else v)


results = {}
for state, cas in CASES.items():
    s = launch()
    try:
        st = s.settings
        st.file.read_case_data(file_name=os.path.join(CFD, cas))
        walls = list(st.setup.boundary_conditions.wall.get_object_names())
        groups = {"louvers": [w for w in walls if "louver" in w], "duct": [w for w in walls if "duct" in w],
                  "rotor": [w for w in walls if "rotor" in w]}
        rd = st.solution.report_definitions
        row = {}
        for g, zones in groups.items():
            nm = f"drag-{g}"
            rd.force.create(nm)
            rd.force[nm].zones = zones
            rd.force[nm].force_vector = [-1.0, 0.0, 0.0]
            row[f"drag_{g}_N"] = value(rd, nm)
            for z in zones:                       # per-zone breakdown (solid-side shadow walls should be ~0)
                zn = f"f-{z}"[:60]
                rd.force.create(zn)
                rd.force[zn].zones = [z]
                rd.force[zn].force_vector = [-1.0, 0.0, 0.0]
                row.setdefault("zones", {})[z] = value(rd, zn)
        row["drag_total_N"] = row["drag_louvers_N"] + row["drag_duct_N"] + row["drag_rotor_N"]
        row["drag_duct_system_N"] = row["drag_louvers_N"] + row["drag_duct_N"]
        surf = st.results.surfaces
        surf.iso_surface.create("throat_x")
        surf.iso_surface["throat_x"].field = "x-coordinate"
        surf.iso_surface["throat_x"].iso_values = [THROAT_X]
        surf.iso_surface["throat_x"].zones = ["fluid_domain"]
        surf.iso_clip.create("throat_y")
        surf.iso_clip["throat_y"].field = "y-coordinate"
        surf.iso_clip["throat_y"].surfaces = ["throat_x"]
        surf.iso_clip["throat_y"].range.minimum = Y0
        surf.iso_clip["throat_y"].range.maximum = Y1
        surf.iso_clip.create("throat")
        surf.iso_clip["throat"].field = "z-coordinate"
        surf.iso_clip["throat"].surfaces = ["throat_y"]
        surf.iso_clip["throat"].range.minimum = Z0
        surf.iso_clip["throat"].range.maximum = Z1
        for nm, rt, fld in (("throat-mdot", "surface-massflowrate", None), ("throat-area", "surface-area", None),
                            ("throat-u", "surface-areaavg", "x-velocity")):
            rd.surface.create(nm)
            rd.surface[nm].report_type = rt
            if fld:
                rd.surface[nm].field = fld
            rd.surface[nm].surface_names = ["throat"]
        row["throat_mass_flow_kg_s"] = -value(rd, "throat-mdot")
        row["throat_area_m2"] = value(rd, "throat-area")
        row["throat_mean_u_m_s"] = -value(rd, "throat-u")
        rd.flux.create("rotor-q")
        rd.flux["rotor-q"].report_type = "flux-heattransfer"
        rd.flux["rotor-q"].boundaries = groups["rotor"]
        row["rotor_heat_W"] = value(rd, "rotor-q")
        rd.flux.create("inlet-mdot")
        rd.flux["inlet-mdot"].report_type = "flux-massflow"
        rd.flux["inlet-mdot"].boundaries = ["inlet"]
        row["inlet_mass_flow_kg_s"] = value(rd, "inlet-mdot")
        results[state] = row
        print("STATE", state, json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()
                                          if k != "zones"}))
        print("ZONES", state, {k: round(v, 4) for k, v in row["zones"].items()})
    finally:
        s.exit()

o, c = results["open"], results["closed"]
summary = {
    "duct_system_drag_change_pct_closing": (c["drag_duct_system_N"] - o["drag_duct_system_N"]) / o["drag_duct_system_N"] * 100,
    "total_drag_change_pct_closing": (c["drag_total_N"] - o["drag_total_N"]) / o["drag_total_N"] * 100,
    "throat_flow_change_pct_closing": (c["throat_mass_flow_kg_s"] - o["throat_mass_flow_kg_s"]) / o["throat_mass_flow_kg_s"] * 100,
    "rotor_heat_change_pct_closing": (c["rotor_heat_W"] - o["rotor_heat_W"]) / o["rotor_heat_W"] * 100,
}
results["summary"] = summary
json.dump(results, open(os.path.join(CFD, "phase2_aero_open_vs_closed.json"), "w"), indent=2)
print("SUMMARY", json.dumps({k: round(v, 2) for k, v in summary.items()}))
