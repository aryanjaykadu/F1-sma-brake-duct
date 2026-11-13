"""Import the KiCad shield STEP into SOLIDWORKS and save it as cad/rig/Sensor_Shield_PCB (assembly or part)."""
from paths import ROOT as PROJECT_ROOT
import json
import os

from swlib import SW, byref_long, get

STEP = PROJECT_ROOT + r"\electronics\kicad\sma_rig_shield.step"
OUT = PROJECT_ROOT + r"\cad\rig"

sw = SW()
err = byref_long()
data = sw.app.GetImportFileData(STEP)
doc = sw.app.LoadFile4(STEP, "r", data, err)
if doc is None:
    raise SystemExit(f"STEP import failed ({err.value})")
kind = get(doc, "GetType")                 # 1 part, 2 assembly
ext = ".SLDASM" if kind == 2 else ".SLDPRT"
path = os.path.join(OUT, "Sensor_Shield_PCB" + ext)
e2, w2 = byref_long(), byref_long()
from swlib import nothing  # noqa: E402
if not doc.Extension.SaveAs(path, 0, 1, nothing(), e2, w2):
    raise SystemExit(f"save failed {e2.value}")
box = doc.GetBox(0) if kind == 2 else get(doc, "GetPartBox")
box = [round(v * 1000, 3) for v in box]
print("TYPE", kind, "BOX mm", box, "size", [round(box[i + 3] - box[i], 2) for i in range(3)])
json.dump({"path": path, "box_mm": box}, open(os.path.join(OUT, "pcb_import.json"), "w"))
doc.ShowNamedView2("*Isometric", 7)
get(doc, "ViewZoomtofit2")
doc.Extension.SaveAs(os.path.join(OUT, "Sensor_Shield_PCB.png"), 0, 3, nothing(), e2, w2)
print("SAVED", path)
