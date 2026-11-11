"""Map air temperature in the saved Case A solution to choose an SMA pocket location."""
from paths import ROOT as PROJECT_ROOT
import os
import sys
import time

import numpy as np
import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
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
try:
    s.settings.file.read_case_data(file_name=os.path.join(CFD, "Phase2_CaseA.cas.h5"))
    sv = s.fields.solution_variable_data
    out = {}
    for var in ("SV_T", "SV_CENTROID"):
        try:
            d = sv.get_data(variable_name=var, zone_names=["fluid_domain"], domain_name="mixture")
            out[var] = np.asarray(d["fluid_domain"])
            print(var, out[var].shape)
        except Exception as exc:  # noqa: BLE001
            print(var, "ERR", str(exc)[:300])
            try:
                info = s.fields.solution_variable_info.get_variables_info(zone_names=["fluid_domain"],
                                                                          domain_name="mixture")
                print("available:", info.solution_variables[:80])
            except Exception as exc2:  # noqa: BLE001
                print("info ERR", str(exc2)[:200])
    if len(out) == 2:
        xyz = out["SV_CENTROID"].reshape(-1, 3)
        np.savez_compressed(os.path.join(CFD, "caseA_air_field.npz"), xyz=xyz, T=out["SV_T"])
        print("saved field", xyz.shape)
finally:
    s.exit()
