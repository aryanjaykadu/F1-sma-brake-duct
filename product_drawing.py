"""Phase 7c - A3 assembly drawing of the SMA duct module (ISO, first angle): 3 views + isometric + BOM -> PDF."""
import os

import product_cad as pc
from swlib import SW, byref_long, call, get, nothing

TEMPLATE = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\data\templates\iso.drwdot"
ASM = os.path.join(pc.OUT, "SMA_Duct_Module.SLDASM")
A3 = 8                                             # swDwgPaperA3size

sw = SW()
sw.app.CloseAllDocuments(True)
err, warn = byref_long(), byref_long()
asm = sw.app.OpenDoc6(ASM, 2, 1, "", err, warn)
if asm is None:
    raise SystemExit("assembly open failed")
for name, val in (("Description", "SMA brake-duct actuator module"), ("Material", "see BOM"),
                  ("Revision", "A"), ("PartNo", "SMA-DM-001")):
    call(asm.Extension.CustomPropertyManager(""), "Add3", name, 30, val, 2)
asm.Save3(1, err, warn)

drw = sw.app.NewDocument(TEMPLATE, A3, 0.42, 0.297)
if drw is None:
    raise SystemExit("drawing create failed")
FORMAT = r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2025\lang\english\sheetformat\a3 - iso.slddrt"
sheet = get(drw, "GetCurrentSheet")
# SetupSheet5(name, paper, template type (12 = custom), scale num, den, first angle, format, w, h, view, keep notes)
print("sheet format", call(drw, "SetupSheet5", get(sheet, "GetName"), A3, 12, 1.0, 2.0, True, FORMAT, 0.42, 0.297,
                           "Default", True))
ok = call(drw, "Create1stAngleViews2", ASM)
print("standard views", ok)
iso = call(drw, "CreateDrawViewFromModelView3", ASM, "*Isometric", 0.21, 0.145, 0.0)
print("iso view", iso is not None)
# scale all views to 1:2 so the 210 mm tall module fits an A3 sheet with the BOM
view = get(drw, "GetFirstView")                   # sheet
view = get(view, "GetNextView")
views = []
while view is not None:
    views.append(view)
    view = get(view, "GetNextView")
for v in views:
    try:
        v.ScaleDecimal = 0.5
    except Exception as exc:  # noqa: BLE001
        print("scale", exc)
drw.ForceRebuild3(False)
# BOM anchored top-right, parts only
bom = None
if views:
    try:
        bom = call(views[0], "InsertBomTable4", False, 0.41, 0.287, 2, 2, "Default", "", False, 0, False)
    except Exception as exc:  # noqa: BLE001
        print("bom", exc)
print("bom", bom is not None)
get(drw, "ViewZoomtofit2")
out = os.path.join(pc.OUT, "SMA-DM-001")
e2, w2 = byref_long(), byref_long()
print("save drw", drw.Extension.SaveAs(out + ".SLDDRW", 0, 1, nothing(), e2, w2), e2.value)
print("save pdf", drw.Extension.SaveAs(out + ".pdf", 0, 1 | 2, nothing(), e2, w2), e2.value)
