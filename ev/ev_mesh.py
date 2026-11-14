"""EV re-target - Fluent Meshing (watertight) for the half-car domain: fluid domain + radiator (+ plenum when closed),
all fluid regions with shared topology. Car faces 40 mm, grille / radiator / plenum 10 mm, global 5-500 mm,
4 prism layers (first 2 mm, wall functions), poly-hexcore. Usage: SHUTTER=open|closed
"""
import json
import os

import ansys.fluent.core as pyfluent

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfd")
OPEN = os.environ.get("SHUTTER", "open").lower() == "open"
NAME = f"EV_Front_{'Open' if OPEN else 'Closed'}"
GEOM, MESH = os.path.join(D, NAME + ".x_t"), os.path.join(D, NAME + ".msh.h5")
labels = json.load(open(os.path.join(D, NAME + ".json")))["labels"]
S_CAR, S_FINE, S_MAX = (float(os.environ.get(k, d)) for k, d in (("S_CAR", "40"), ("S_FINE", "10"), ("S_MAX", "500")))


def state(task):
    try:
        return task.Arguments.get_state()
    except Exception:  # noqa: BLE001
        return {}


m = pyfluent.launch_fluent(mode=pyfluent.FluentMode.MESHING, precision=pyfluent.Precision.DOUBLE,
                           dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=D)
try:
    wf, mu = m.workflow, m.meshing_utilities
    wf.InitializeWorkflow(WorkflowType="Watertight Geometry")
    t = wf.TaskObject["Import Geometry"]
    t.Arguments.set_state({"FileName": GEOM, "LengthUnit": "mm"})
    t.Execute()
    mapping = {}
    for obj in mu.get_all_objects():
        lbs = mu.get_labels(object_name=obj) or []
        if "inlet" in lbs:
            name = "fluid_domain"
        else:
            tags = [lb[3:] for lb in lbs if lb.startswith("id_")]
            name = tags[0] if len(tags) == 1 else None
        print("OBJ", obj, "->", name, flush=True)
        if name is None or name in mapping.values():
            raise RuntimeError(f"cannot classify {obj}: {lbs[:8]}")
        mu.rename_object(old_object_name=obj, new_object_name=name)
        for lb in [lb for lb in lbs if lb.startswith("region")]:
            mu.rename_face_zone_label(object_name=name, old_label_name=lb, new_label_name=name)
        mapping[obj] = name
    fine = [lb for k, v in labels.items() if k in ("radiator", "radiator_if", "plenum", "plenum_if") for lb in v]
    for nm, labs, size in (("car", labels["car"], S_CAR), ("grille_radiator", fine, S_FINE)):
        t = wf.TaskObject["Add Local Sizing"]
        t.Arguments.set_state({"AddChild": "yes", "BOIControlName": nm, "BOIExecution": "Face Size",
                               "BOIFaceLabelList": labs, "BOISize": size, "BOIGrowthRate": 1.2})
        t.AddChildAndUpdate(DeferUpdate=False)
    t = wf.TaskObject["Generate the Surface Mesh"]
    t.Arguments.set_state({"CFDSurfaceMeshControls": {"MinSize": 5.0, "MaxSize": S_MAX, "GrowthRate": 1.2,
                                                      "SizeFunctions": "Curvature & Proximity"}})
    t.Execute()
    t = wf.TaskObject["Describe Geometry"]
    t.UpdateChildTasks(SetupTypeChanged=False)
    t.Arguments.set_state({"SetupType": "The geometry consists of both fluid and solid regions and/or voids",
                           "InvokeShareTopology": "Yes", "WallToInternal": "Yes"})
    t.UpdateChildTasks(SetupTypeChanged=True)
    t.Execute()
    if "Apply Share Topology" in wf.TaskObject.get_object_names():
        wf.TaskObject["Apply Share Topology"].Execute()
    sym = labels["symmetry"] + labels["farfield"]
    t = wf.TaskObject["Update Boundaries"]
    t.Arguments.set_state({"BoundaryLabelList": ["inlet", "outlet"] + sym,
                           "BoundaryLabelTypeList": ["velocity-inlet", "pressure-outlet"] + ["symmetry"] * len(sym),
                           "OldBoundaryLabelList": ["inlet", "outlet"] + sym,
                           "OldBoundaryLabelTypeList": ["velocity-inlet", "pressure-outlet"] + ["wall"] * len(sym)})
    t.Execute()
    if "Create Regions" in wf.TaskObject.get_object_names():
        try:
            t = wf.TaskObject["Create Regions"]
            t.Arguments.set_state({"NumberOfFlowVolumes": 0})
            t.Execute()
        except Exception as exc:  # noqa: BLE001
            print("Create Regions:", exc)
    t = wf.TaskObject["Update Regions"]
    region_map = {obj.replace("origin-object", "region"): name for obj, name in mapping.items()}
    old = list(region_map)
    t.Arguments.set_state({"OldRegionNameList": old, "OldRegionTypeList": ["solid"] * len(old),
                           "RegionNameList": [region_map[o] for o in old], "RegionTypeList": ["fluid"] * len(old)})
    t.Execute()
    print("REGIONS", region_map, flush=True)
    t = wf.TaskObject["Add Boundary Layers"]
    t.Arguments.set_state({"AddChild": "yes", "BLControlName": "car_prisms", "OffsetMethodType": "uniform",
                           "NumberOfLayers": 4, "FirstHeight": 2.0, "Rate": 1.3})
    t.AddChildAndUpdate(DeferUpdate=False)
    t = wf.TaskObject["Generate the Volume Mesh"]
    t.Arguments.set_state({"VolumeFill": "poly-hexcore", "VolumeFillControls": {"HexMaxCellLength": S_MAX}})
    t.Execute()
    m.tui.mesh.check_mesh()
    if os.path.exists(MESH):
        os.remove(MESH)
    m.meshing.File.WriteMesh(FileName=MESH)
    print("MESH WRITTEN", MESH, flush=True)
finally:
    m.exit()
