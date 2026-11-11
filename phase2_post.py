"""Phase 2c - export contour images from the saved Case A solution using Fluent graphics."""
from paths import ROOT as PROJECT_ROOT
import os
import sys
import time

import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
TAG = os.environ.get("TAG", "CaseA")
CAS = os.path.join(CFD, f"Phase2_{TAG}.cas.h5")

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
    sys.exit("Fluent launch failed")
try:
    st = s.settings
    st.file.read_case_data(file_name=CAS)
    surf = st.results.surfaces.plane_surface
    surf.create("mid_z")
    surf["mid_z"].method = "xy-plane"
    surf["mid_z"].z = 0.0
    surf.create("pocket_z")
    surf["pocket_z"].method = "xy-plane"
    surf["pocket_z"].z = 0.004          # plane through the relocated pocket, plotted on a 300-400 K scale
    g = st.results.graphics
    try:
        g.picture.use_window_resolution = False
    except Exception as exc:  # noqa: BLE001
        print("use_window_resolution not set:", exc)
    g.picture.x_resolution = 1920
    g.picture.y_resolution = 1080
    g.picture.driver_options.hardcopy_format = "png"
    for name, field, surface, fname, rng in (
            ("temp_mid", "temperature", "mid_z", f"Temp_Contour_{TAG}.png", (300.0, 1073.15)),
            ("temp_pocket", "temperature", "pocket_z", f"Temp_Contour_{TAG}_PocketPlane.png", (300.0, 400.0)),
            ("vel_mid", "velocity-magnitude", "mid_z", f"Velocity_Contour_{TAG}.png", None)):
        g.contour.create(name)
        c = g.contour[name]
        c.field = field
        c.surfaces_list = [surface]
        if rng:
            try:
                c.range_option.option = "auto-range-off"
                c.range_option.auto_range_off.minimum = rng[0]
                c.range_option.auto_range_off.maximum = rng[1]
                c.range_option.auto_range_off.clip_to_range = False
            except Exception as exc:  # noqa: BLE001
                print("range not set:", exc)
        c.display()
        g.views.restore_view(view_name="front")
        g.views.auto_scale()
        out = os.path.join(CFD, fname)
        if os.path.exists(out):
            os.remove(out)
        g.picture.save_picture(file_name=out)
        print("saved", out, os.path.isfile(out))
finally:
    s.exit()
