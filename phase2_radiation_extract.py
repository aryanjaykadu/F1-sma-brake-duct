"""Area-averaged incident radiation, air temperature and velocity on planes inside the SMA pocket at set
distances from the rotor's outboard face (x = -16 mm). Uses Fluent iso-surfaces restricted to the pocket
zone and surface report definitions (the per-cell SV_DO_IRRAD export returns uninitialised data).
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
TAG = os.environ.get('TAG', 'CaseA')
import os
import sys
import time

import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
GAPS_MM = [0.5, 1, 2, 3, 4, 6, 8, 10, 12, 15]
s = None
for i in range(5):
    try:
        s = pyfluent.launch_fluent(mode=pyfluent.FluentMode.SOLVER, precision=pyfluent.Precision.DOUBLE,
                                   dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=CFD)
        break
    except Exception as exc:  # noqa: BLE001
        print("launch retry", i + 1, type(exc).__name__)
        time.sleep(45)
if s is None:
    sys.exit("launch failed")
rows = []
try:
    st = s.settings
    st.file.read_case_data(file_name=os.path.join(CFD, f"Phase2_{TAG}.cas.h5"))
    iso = st.results.surfaces.iso_surface
    rd = st.solution.report_definitions
    for gap in GAPS_MM:
        nm = f"gap_{str(gap).replace('.', 'p')}"
        iso.create(nm)
        iso[nm].field = "x-coordinate"
        iso[nm].iso_values = [-(16.0 + gap) / 1000.0]
        iso[nm].zones = ["sma_pocket"]
        row = {"gap_mm": gap}
        for key, field in (("G_W_m2", "incident-radiation"), ("T_air_K", "temperature"),
                           ("U_m_s", "velocity-magnitude")):
            rn = f"{nm}_{key}".lower()
            rd.surface.create(rn)
            rd.surface[rn].report_type = "surface-areaavg"
            rd.surface[rn].field = field
            rd.surface[rn].surface_names = [nm]
            val = rd.compute(report_defs=[rn])[0][rn]
            row[key] = float(val[0] if isinstance(val, (list, tuple)) else val)
        rows.append(row)
        print("ROW", row)
    rn = "pocket_g_vol"
    rd.volume.create(rn)
    rd.volume[rn].report_type = "volume-average"
    rd.volume[rn].field = "incident-radiation"
    rd.volume[rn].cell_zones = ["sma_pocket"]
    g_vol = rd.compute(report_defs=[rn])[0][rn]
    print("POCKET G vol-avg", g_vol)
finally:
    s.exit()
with open(os.path.join(CFD, f"pocket_planes_{TAG}.json"), "w") as fh:
    json.dump(rows, fh, indent=2)
print("saved", len(rows), "planes")
