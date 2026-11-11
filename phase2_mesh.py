"""Phase 2a - Fluent Meshing (Watertight Geometry) for the CHT domain built in SOLIDWORKS.

Sized to stay inside the ANSYS Student 1M-cell limit:
  surface 0.5-10 mm, 1.0 mm face size on louvers + SMA pocket,
  10 uniform prism layers (first height 0.02 mm, growth 1.2), poly-hexcore volume fill.
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import os
import sys

import ansys.fluent.core as pyfluent

CFD = PROJECT_ROOT + r"\cfd"
SUFFIX = os.environ.get("SUFFIX", "")
GEOM = os.path.join(CFD, f"Phase2_CHT_Domain{SUFFIX}.x_t")
MESH = os.path.join(CFD, f"Phase2_CHT_Domain{SUFFIX}.msh.h5")
H_CY, PITCH = 140.0, 20.0
FLUIDS = ("fluid_domain", "sma_pocket")
LOCAL_SIZE = float(os.environ.get("LOCAL_SIZE", "1.0"))
MAX_SIZE = float(os.environ.get("MAX_SIZE", "10.0"))
BL_LAYERS = int(os.environ.get("BL_LAYERS", "10"))


def classify(center):
    x, y, z = center
    if abs(x - 66.5) < 2 and z > 25:
        return "sma_pocket"
    if abs(x) < 2:
        return "rotor"
    if abs(x - 58.0) < 2 and abs(z) < 2:
        if abs(y - H_CY) < 3:
            return "duct_wall"
        k = int(round((y - H_CY) / PITCH + 1.5)) + 1
        return f"louver_{k}"
    return None


def state(task):
    try:
        return task.Arguments.get_state()
    except Exception:  # noqa: BLE001
        return {}


def main():
    m = pyfluent.launch_fluent(mode=pyfluent.FluentMode.MESHING, precision=pyfluent.Precision.DOUBLE,
                               dimension=pyfluent.Dimension.THREE, processor_count=4, cwd=CFD)
    report = {}
    try:
        wf = m.workflow
        mu = m.meshing_utilities
        wf.InitializeWorkflow(WorkflowType="Watertight Geometry")
        t = wf.TaskObject["Import Geometry"]
        t.Arguments.set_state({"FileName": GEOM, "LengthUnit": "mm"})
        t.Execute()

        # ---- identify and rename objects / labels ----
        mapping = {}
        for obj in mu.get_all_objects():
            labels = mu.get_labels(object_name=obj) or []
            zones = mu.get_face_zones_of_object(object_name=obj)
            if "inlet" in labels:
                name = "fluid_domain"
            else:
                tags = [lb[3:] for lb in labels if lb.startswith("id_")]
                name = tags[0] if len(tags) == 1 else None
                print("OBJ", obj, labels, "->", name)
            if name is None or name in mapping.values():
                raise RuntimeError(f"could not classify {obj} ({name})")
            region_label = [lb for lb in labels if lb.startswith("region")]
            mu.rename_object(old_object_name=obj, new_object_name=name)
            for lb in region_label:
                mu.rename_face_zone_label(object_name=name, old_label_name=lb, new_label_name=name)
            mapping[obj] = name
        print("MAPPING", mapping)
        print("LABELS", {o: mu.get_labels(object_name=o) for o in mu.get_all_objects()})
        report["objects"] = mapping

        # ---- local sizing ----
        t = wf.TaskObject["Add Local Sizing"]
        t.Arguments.set_state({"AddChild": "yes", "BOIControlName": "louver_pocket_size", "BOIExecution": "Face Size",
                               "BOIFaceLabelList": [f"{p}louver_{k}" for k in range(1, 5) for p in ("", "id_")]
                                                   + ["sma_pocket", "id_sma_pocket"],
                               "BOISize": LOCAL_SIZE, "BOIGrowthRate": 1.2})
        try:
            t.AddChildAndUpdate(DeferUpdate=False)
        except Exception as exc:  # noqa: BLE001
            print("AddChildAndUpdate failed, fallback:", exc)
            t.AddChildToTask()
            t.Execute()

        # ---- surface mesh ----
        t = wf.TaskObject["Generate the Surface Mesh"]
        t.Arguments.set_state({"CFDSurfaceMeshControls": {"MinSize": 0.5, "MaxSize": MAX_SIZE, "GrowthRate": 1.2,
                                                          "SizeFunctions": "Curvature & Proximity"}})
        t.Execute()

        # ---- describe geometry ----
        t = wf.TaskObject["Describe Geometry"]
        t.UpdateChildTasks(SetupTypeChanged=False)
        t.Arguments.set_state({"SetupType": "The geometry consists of both fluid and solid regions and/or voids",
                               "InvokeShareTopology": "Yes", "WallToInternal": "Yes"})
        t.UpdateChildTasks(SetupTypeChanged=True)
        t.Execute()
        names = wf.TaskObject.get_object_names()
        print("TASKS", names)
        if "Apply Share Topology" in names:
            wf.TaskObject["Apply Share Topology"].Execute()

        t = wf.TaskObject["Update Boundaries"]
        print("UB state", state(t))
        labels = ["inlet", "outlet"] + [f"farfield{i}" for i in range(5)]
        types = ["velocity-inlet", "pressure-outlet"] + ["symmetry"] * 5
        t.Arguments.set_state({"BoundaryLabelList": labels, "BoundaryLabelTypeList": types,
                               "OldBoundaryLabelList": labels,
                               "OldBoundaryLabelTypeList": ["velocity-inlet", "pressure-outlet"] + ["wall"] * 5})
        t.Execute()

        if "Create Regions" in wf.TaskObject.get_object_names():
            t = wf.TaskObject["Create Regions"]
            print("CR state", state(t))
            try:
                t.Arguments.set_state({"NumberOfFlowVolumes": 0})
                t.Execute()
            except Exception as exc:  # noqa: BLE001
                print("Create Regions (0 volumes) failed:", exc)

        t = wf.TaskObject["Update Regions"]
        st = state(t)
        print("UR state", st)
        rnames = list(st.get("RegionNameList") or st.get("OldRegionNameList") or [])
        rtypes = list(st.get("RegionTypeList") or st.get("OldRegionTypeList") or [])
        region_map = {obj.replace("origin-object", "region"): name for obj, name in mapping.items()}
        old = list(region_map)
        new = [region_map[o] for o in old]
        desired = ["fluid" if n in FLUIDS else "solid" for n in new]
        t.Arguments.set_state({"OldRegionNameList": old, "OldRegionTypeList": ["solid"] * len(old),
                               "RegionNameList": new, "RegionTypeList": desired})
        t.Execute()
        print("REGIONS", dict(zip(old, zip(new, desired))))

        t = wf.TaskObject["Add Boundary Layers"]
        t.Arguments.set_state({"AddChild": "yes", "BLControlName": "cht_prisms", "OffsetMethodType": "uniform",
                               "NumberOfLayers": BL_LAYERS, "FirstHeight": 0.02, "Rate": 1.2})
        try:
            t.AddChildAndUpdate(DeferUpdate=False)
        except Exception as exc:  # noqa: BLE001
            print("BL AddChildAndUpdate failed, fallback:", exc)
            t.AddChildToTask()
            t.Execute()

        t = wf.TaskObject["Generate the Volume Mesh"]
        t.Arguments.set_state({"VolumeFill": "poly-hexcore", "VolumeFillControls": {"HexMaxCellLength": MAX_SIZE}})
        t.Execute()

        cells = {z: mu.get_cell_zone_count(cell_zone_name=z) if False else None for z in []}
        m.tui.mesh.check_mesh()
        try:
            m.tui.report.cell_zone_volume()
        except Exception:  # noqa: BLE001
            pass
        if os.path.exists(MESH):
            os.remove(MESH)
        m.meshing.File.WriteMesh(FileName=MESH)
        report["mesh"] = MESH
        report["local_size_mm"] = LOCAL_SIZE
        report["max_size_mm"] = MAX_SIZE
        report["bl_layers"] = BL_LAYERS
        with open(os.path.join(CFD, f"phase2_mesh{SUFFIX}.json"), "w") as fh:
            json.dump(report, fh, indent=2)
        print("MESH WRITTEN", MESH)
    finally:
        m.exit()


if __name__ == "__main__":
    sys.exit(main())
