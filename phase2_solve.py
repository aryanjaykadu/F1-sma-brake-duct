"""Phase 2b - Fluent CHT solve, Case A: 100 km/h, 800 C rotor, louvers closed.

Energy + k-omega SST + Discrete Ordinates radiation, incompressible-ideal-gas air,
carbon-carbon rotor, CFRP duct, Nitinol louvers. Reports the volume-average air temperature
in the SMA pocket and checks it against the 358 K activation threshold.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import fnmatch
import json
import os
import sys
import time

import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
SUFFIX = os.environ.get("SUFFIX", "")
MESH = os.path.join(CFD, f"Phase2_CHT_Domain{SUFFIX}.msh.h5")
ITER = int(os.environ.get("ITER", "500"))
TAG = os.environ.get("TAG", "CaseA")
CASE = dict(key="A", title="Corner entry / heavy braking", inlet_velocity=27.78, inlet_temperature=300.0,
            outlet_gauge_pressure=0.0, rotor_temperature=1073.15, louver_state="CLOSED (0 deg)",
            iterations=ITER, t_pocket_min=358.0)
if os.environ.get("CASE_KEY", "A") == "B":
    CASE = dict(key="B", title="Straight-line / high speed", inlet_velocity=83.33, inlet_temperature=300.0,
                outlet_gauge_pressure=0.0, rotor_temperature=673.15, louver_state="OPEN (90 deg)",
                iterations=ITER, t_pocket_min=358.0)
if os.environ.get("CASE_KEY", "A") == "B_closed":
    CASE = dict(key="B_closed", title="Straight-line / high speed, louvers closed", inlet_velocity=83.33,
                inlet_temperature=300.0, outlet_gauge_pressure=0.0, rotor_temperature=673.15,
                louver_state="CLOSED (0 deg)", iterations=ITER, t_pocket_min=358.0)
SOLIDS = {"carbon-carbon": (1800.0, 40.0, 800.0, 0.85, ("*rotor*",)),
          "cfrp": (1550.0, 5.0, 1100.0, 0.90, ("*duct*",)),
          "nitinol": (6450.0, 18.0, 837.0, 0.60, ("*louver*",))}
notes = []


def attempt(label, *fns, required=True):
    errs = []
    for fn in fns:
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001
            errs.append(f"{type(exc).__name__}: {str(exc)[:160]}")
    msg = f"{label}: " + " | ".join(errs)
    if required:
        raise RuntimeError(msg)
    print("WARN", msg)
    notes.append(msg)
    return None


def names(container):
    try:
        return list(container.get_object_names())
    except Exception:  # noqa: BLE001
        return []


def main():
    t0 = time.time()
    s = None
    for i in range(5):
        try:
            s = pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                       dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=CFD)
            break
        except Exception as exc:  # noqa: BLE001 - licence/session release lag after a previous run
            print(f"launch attempt {i + 1} failed: {type(exc).__name__}; retrying in 45 s")
            time.sleep(45)
    if s is None:
        raise RuntimeError("Fluent could not be launched after 5 attempts")
    result = dict(key=CASE["key"], title=CASE["title"], louver_state=CASE["louver_state"], status="NOT RUN")
    try:
        st = s.settings
        st.file.read_mesh(file_name=MESH)
        attempt("improve quality", lambda: st.mesh.repair_improve.improve_quality(), required=False)
        attempt("mesh check", lambda: st.mesh.check(), required=False)
        bc, cz = st.setup.boundary_conditions, st.setup.cell_zone_conditions
        inv = {k: names(getattr(bc, k)) for k in ("wall", "velocity_inlet", "pressure_outlet", "symmetry", "interior")}
        fluids, solids = names(cz.fluid), names(cz.solid)
        print("ZONES fluid", fluids, "solid", solids)
        for k, v in inv.items():
            print("BC", k, v)
        if "sma_pocket" not in fluids or "inlet" not in inv["velocity_inlet"] or "outlet" not in inv["pressure_outlet"]:
            raise RuntimeError("zone contract violated")

        m = st.setup.models
        m.energy.enabled = True
        m.viscous.model = "k-omega"
        m.viscous.k_omega_model = "sst"
        attempt("DO model", lambda: setattr(m.radiation, "model", "discrete-ordinates"),
                lambda: setattr(m.radiation, "model", "do"))
        attempt("DO divisions",
                lambda: (setattr(m.radiation.discrete_ordinates.angular_discretization, "theta_divisions", 2),
                         setattr(m.radiation.discrete_ordinates.angular_discretization, "phi_divisions", 2)),
                required=False)

        mats = st.setup.materials
        air = mats.fluid["air"]
        attempt("air density", lambda: setattr(air.density, "option", "incompressible-ideal-gas"))
        attempt("air cp", lambda: (setattr(air.specific_heat, "option", "constant"),
                                   setattr(air.specific_heat, "value", 1006.43)))
        attempt("air k", lambda: (setattr(air.thermal_conductivity, "option", "constant"),
                                  setattr(air.thermal_conductivity, "value", 0.0242)))
        assign = {}
        for name, (rho, k, cp, _, pats) in SOLIDS.items():
            attempt(f"create {name}", lambda n=name: mats.solid.create(n), lambda n=name: mats.solid.__setitem__(n, {}))
            sm = mats.solid[name]
            for prop, val in (("density", rho), ("thermal_conductivity", k), ("specific_heat", cp)):
                attempt(f"{name}.{prop}", lambda p=prop, v=val: (setattr(getattr(sm, p), "option", "constant"),
                                                               setattr(getattr(sm, p), "value", v)),
                        lambda p=prop, v=val: setattr(getattr(sm, p), "value", v))
            for z in solids:
                if any(fnmatch.fnmatch(z, p) for p in pats):
                    attempt(f"{z} material", lambda z=z, n=name: setattr(cz.solid[z].general, "material", n),
                            lambda z=z, n=name: setattr(cz.solid[z], "material", n))
                    assign[z] = name
        print("MATERIALS", assign)
        if len(assign) != len(solids):
            raise RuntimeError(f"unassigned solid zones: {set(solids) - set(assign)}")

        inlet = bc.velocity_inlet["inlet"]
        attempt("inlet v", lambda: setattr(inlet.momentum.velocity_magnitude, "value", CASE["inlet_velocity"]),
                lambda: setattr(inlet.momentum.velocity, "value", CASE["inlet_velocity"]))
        attempt("inlet T", lambda: setattr(inlet.thermal.temperature, "value", CASE["inlet_temperature"]))
        outlet = bc.pressure_outlet["outlet"]
        attempt("outlet p", lambda: setattr(outlet.momentum.gauge_pressure, "value", 0.0))
        attempt("outlet T", lambda: setattr(outlet.thermal.backflow_total_temperature, "value", 300.0), required=False)

        rotor_walls = [w for w in inv["wall"] if "rotor" in w]
        print("ROTOR WALLS", rotor_walls)
        if not rotor_walls:
            raise RuntimeError("no rotor walls")
        for w in rotor_walls:
            wall = bc.wall[w]
            attempt(f"{w} thermal", lambda wall=wall: setattr(wall.thermal, "thermal_condition", "Temperature"),
                    lambda wall=wall: setattr(wall.thermal, "thermal_bc", "Temperature"))
            attempt(f"{w} T", lambda wall=wall: setattr(wall.thermal.temperature, "value", CASE["rotor_temperature"]),
                    lambda wall=wall: setattr(wall.thermal.t, "value", CASE["rotor_temperature"]))
        for name, (_, _, _, emis, pats) in SOLIDS.items():
            for w in inv["wall"]:
                if any(fnmatch.fnmatch(w, p) for p in pats):
                    attempt(f"emissivity {w}",
                            lambda w=w, e=emis: setattr(bc.wall[w].radiation.internal_emissivity, "value", e),
                            lambda w=w, e=emis: setattr(bc.wall[w].radiation, "internal_emissivity", e), required=False)

        attempt("coupled", lambda: setattr(st.solution.methods.p_v_coupling, "flow_scheme", "Coupled"), required=False)
        eqs = st.solution.monitor.residual.equations
        print("RESIDUAL EQS", names(eqs))
        for eq in names(eqs):
            crit = 1e-5 if eq in ("continuity", "x-velocity", "y-velocity", "z-velocity") else \
                1e-6 if eq in ("energy", "do-intensity") else None
            if crit:
                attempt(f"residual {eq}", lambda e=eq, c=crit: setattr(eqs[e], "absolute_criteria", c), required=False)

        rd = st.solution.report_definitions
        for nm, rt in (("t-pocket-avg", "volume-average"), ("t-pocket-max", "volume-max")):
            rd.volume.create(nm)
            rd.volume[nm].report_type = rt
            rd.volume[nm].field = "temperature"
            rd.volume[nm].cell_zones = ["sma_pocket"]
        rd.surface.create("t-outlet-avg")
        rd.surface["t-outlet-avg"].report_type = "surface-massavg"
        rd.surface["t-outlet-avg"].field = "temperature"
        rd.surface["t-outlet-avg"].surface_names = ["outlet"]
        rd.flux.create("rotor-heat")
        attempt("rotor heat report", lambda: (setattr(rd.flux["rotor-heat"], "report_type", "flux-heattransfer"),
                                              setattr(rd.flux["rotor-heat"], "boundaries", rotor_walls)), required=False)

        st.solution.initialization.hybrid_initialize()
        print(f"ITERATING {ITER}")
        st.solution.run_calculation.iterate(iter_count=ITER)

        vals = {}
        for nm in ("t-pocket-avg", "t-pocket-max", "t-outlet-avg", "rotor-heat"):
            r = attempt(f"compute {nm}", lambda nm=nm: rd.compute(report_defs=[nm]), required=nm.startswith("t-pocket"))
            if r:
                d = r[0] if isinstance(r, list) else r
                v = d[nm]
                vals[nm] = float(v[0] if isinstance(v, (list, tuple)) else v)
        print("REPORTS", vals)
        result.update(t_pocket_avg=vals["t-pocket-avg"], t_pocket_max=vals["t-pocket-max"],
                      t_outlet_massavg=vals.get("t-outlet-avg"), rotor_heat_W=vals.get("rotor-heat"))
        ok = vals["t-pocket-avg"] >= CASE["t_pocket_min"]
        result["t_pocket_check"] = f"{'PASS' if ok else 'FAIL'} ({vals['t-pocket-avg']:.2f} K {'>=' if ok else '<'} 358.0 K)"
        cas = os.path.join(CFD, f"Phase2_{TAG}.cas.h5")
        for f in (cas, cas.replace(".cas.h5", ".dat.h5")):
            if os.path.exists(f):
                os.remove(f)
        st.file.write(file_type="case-data", file_name=cas)
        result.update(status="COMPLETED", case_file=cas, iterations=ITER)
    except Exception as exc:  # noqa: BLE001
        result["status"] = f"FAILED: {type(exc).__name__}"
        result["notes"] = [str(exc)[:500]]
        print("FAILED", exc)
    finally:
        result["wall_time_s"] = time.time() - t0
        result.setdefault("notes", []).extend(notes)
        try:
            s.exit()
        except Exception:  # noqa: BLE001
            pass
    cells = None
    try:
        import re
        log = open(os.path.join(CFD, f"phase2_mesh{SUFFIX}.log"), encoding="utf-8", errors="replace").read()
        cells = int(re.findall(r"(\d+) cells were created", log)[-1])
    except Exception:  # noqa: BLE001
        pass
    with open(os.path.join(CFD, "phase2_cht_results.json" if TAG == "CaseA" else f"phase2_{TAG}.json"), "w") as fh:
        json.dump({"cases": [CASE], "results": [result], "mesh_cells": cells}, fh, indent=2)
    print("RESULT", json.dumps(result, indent=1))
    return 0 if result["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
