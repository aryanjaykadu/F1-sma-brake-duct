"""Probe: list every boundary zone of the EV mesh by type, with each zone's area-weighted centroid."""
import os
import sys
import time

import ansys.fluent.core as pyfluent

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfd")
VAR = sys.argv[1] if len(sys.argv) > 1 else "Open"
for k in range(5):
    try:
        s = pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                   dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=D)
        break
    except Exception:  # noqa: BLE001
        time.sleep(60)
try:
    st = s.settings
    st.file.read_mesh(file_name=os.path.join(D, f"EV_Front_{VAR}.msh.h5"))
    bc = st.setup.boundary_conditions
    for typ in bc.child_names:
        obj = getattr(bc, typ)
        try:
            names = list(obj.get_object_names())
        except Exception:  # noqa: BLE001
            continue
        if names:
            print("TYPE", typ, names, flush=True)
    rd = st.solution.report_definitions
    zones = []
    for typ in ("wall", "interior", "symmetry"):
        try:
            zones += list(getattr(bc, typ).get_object_names())
        except Exception:  # noqa: BLE001
            pass
    for z in zones:
        if z.startswith(("radiator", "id_radiator", "plenum", "id_plenum", "interior", "wall")):
            vals = []
            for f in ("x-coordinate", "y-coordinate", "z-coordinate"):
                nm = f"c{len(vals)}-{z}"[:40]
                rd.surface.create(nm)
                rd.surface[nm].report_type = "surface-areaavg"
                rd.surface[nm].field = f
                rd.surface[nm].surface_names = [z]
                v = rd.compute(report_defs=[nm])[0][nm]
                vals.append(round(float(v[0] if isinstance(v, (list, tuple)) else v), 4))
            rd.surface.create(f"a-{z}"[:40])
            rd.surface[f"a-{z}"[:40]].report_type = "surface-area"
            rd.surface[f"a-{z}"[:40]].surface_names = [z]
            a = rd.compute(report_defs=[f"a-{z}"[:40]])[0][f"a-{z}"[:40]]
            print("ZONE", z, "centroid", vals, "area", round(float(a[0] if isinstance(a, (list, tuple)) else a), 5),
                  flush=True)
finally:
    s.exit()
