"""Phase 2g - Fluent Meshing (Watertight, fluid only) for the isolated-wheel aero domain.

Local face sizes: louvers 1.5 mm, duct 4 mm, wheel / cover / rotor 12 mm; global 1-100 mm;
5 uniform prism layers (first height 0.1 mm); poly-hexcore. Sized for the ANSYS Student 1M-cell limit.
Usage: VARIANT=Closed|Open
"""
from paths import ROOT as PROJECT_ROOT
import json
import os
import re
import sys

import ansys.fluent.core as pyfluent

D = PROJECT_ROOT + r"\cfd\wheel"
VARIANT = os.environ.get("VARIANT", "Closed")
NAME = f"Wheel_Aero_{VARIANT}"
GEOM, MESH = os.path.join(D, NAME + ".x_t"), os.path.join(D, NAME + ".msh.h5")
SIZES = {"louver": float(os.environ.get("S_LOUVER", "1.5")), "duct": float(os.environ.get("S_DUCT", "4")),
         "body": float(os.environ.get("S_BODY", "12"))}
MAX_SIZE = float(os.environ.get("MAX_SIZE", "100"))
labels = json.load(open(os.path.join(D, NAME + ".json")))["labels"]


def add_sizing(wf, name, face_labels, size):
    t = wf.TaskObject["Add Local Sizing"]
    t.Arguments.set_state({"AddChild": "yes", "BOIControlName": name, "BOIExecution": "Face Size",
                           "BOIFaceLabelList": face_labels, "BOISize": size, "BOIGrowthRate": 1.2})
    t.AddChildAndUpdate(DeferUpdate=False)


m = pyfluent.launch_fluent(mode=pyfluent.FluentMode.MESHING, precision=pyfluent.Precision.DOUBLE,
                           dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=D)
try:
    wf = m.workflow
    wf.InitializeWorkflow(WorkflowType="Watertight Geometry")
    t = wf.TaskObject["Import Geometry"]
    t.Arguments.set_state({"FileName": GEOM, "LengthUnit": "mm"})
    t.Execute()
    print("LABELS", m.meshing_utilities.get_labels(object_name=m.meshing_utilities.get_all_objects()[0]))
    add_sizing(wf, "louvers", [lb for k, v in labels.items() if k.startswith("louver") for lb in v], SIZES["louver"])
    add_sizing(wf, "duct", labels["duct"], SIZES["duct"])
    add_sizing(wf, "body", labels["wheel"] + labels["cover"] + labels["rotor"], SIZES["body"])
    t = wf.TaskObject["Generate the Surface Mesh"]
    t.Arguments.set_state({"CFDSurfaceMeshControls": {"MinSize": 1.0, "MaxSize": MAX_SIZE, "GrowthRate": 1.2,
                                                      "SizeFunctions": "Curvature"}})
    t.Execute()
    t = wf.TaskObject["Describe Geometry"]
    t.UpdateChildTasks(SetupTypeChanged=False)
    t.Arguments.set_state({"SetupType": "The geometry consists of only fluid regions with no voids"})
    t.UpdateChildTasks(SetupTypeChanged=True)
    t.Execute()
    sym = labels["ground"] + labels["farfield"]
    t = wf.TaskObject["Update Boundaries"]
    t.Arguments.set_state({"BoundaryLabelList": ["inlet", "outlet"] + sym,
                           "BoundaryLabelTypeList": ["velocity-inlet", "pressure-outlet"] + ["symmetry"] * len(sym),
                           "OldBoundaryLabelList": ["inlet", "outlet"] + sym,
                           "OldBoundaryLabelTypeList": ["velocity-inlet", "pressure-outlet"] + ["wall"] * len(sym)})
    t.Execute()
    wf.TaskObject["Update Regions"].Execute()
    t = wf.TaskObject["Add Boundary Layers"]
    t.Arguments.set_state({"AddChild": "yes", "BLControlName": "aero_prisms", "OffsetMethodType": "uniform",
                           "NumberOfLayers": int(os.environ.get("BL_LAYERS", "5")), "FirstHeight": 0.1, "Rate": 1.25})
    t.AddChildAndUpdate(DeferUpdate=False)
    t = wf.TaskObject["Generate the Volume Mesh"]
    t.Arguments.set_state({"VolumeFill": "poly-hexcore", "VolumeFillControls": {"HexMaxCellLength": MAX_SIZE}})
    t.Execute()
    m.tui.mesh.check_mesh()
    if os.path.exists(MESH):
        os.remove(MESH)
    m.meshing.File.WriteMesh(FileName=MESH)
    print("MESH WRITTEN", MESH)
finally:
    m.exit()
