# Passive SMA Brake Duct: feasibility study

Can a brake-cooling duct open and close itself, with no electronics, using a
shape-memory-alloy (Nitinol) spring heated by the brake disc?

The idea: louvers in the duct stay shut on the straights to cut drag, and a
spring heated by the rotor opens them after heavy braking.
I modelled the mechanism in SOLIDWORKS, ran thermal and aero CFD in ANSYS
Fluent, checked the structure in ANSYS MAPDL, and tested the actuator
behaviour over a simulated lap.

![Assembly](cad/Phase1_ThermoKinetic_Assembly.png)

## What I found

| Question | Answer | Evidence |
|---|---|---|
| Does the linkage give enough stroke? | Yes: 14.0 mm rod travel for 80° of louver rotation | SOLIDWORKS mates, `cad/` |
| Is air near the rotor hot enough to trigger the spring? | No: 300-324 K against the 358 K needed | Fluent, `cfd/Temp_Contour_CaseA*.png` |
| Can rotor radiation trigger it? | Yes, if the wire is dark (emissivity ≥ ~0.45) | 119-129 kW/m² incident, `cfd/SMA_Wire_Temperature_CaseA.png` |
| Does it close again on the straight? | Yes: wire settles at 306-311 K | Fluent Case B |
| Does the specified 1.2 mm NiTi spring work over a lap? | No. It is too slow to heat and its hysteresis is too wide | `cfd/transient/` |
| Redesign | Six 0.5 mm NiTiCu springs (Af 76 °C, Mf 58 °C). They open after the 3 hardest braking zones and are open 2 % of straight-line time | `cfd/transient/`, `fea/pack_NiTiCu/` |
| Stresses OK? | Yes, lowest factor of safety 3.3 | MAPDL, `fea/` |
| Does closing the louvers cut drag by 18.4 %, as the brief assumed? | **No, about 1.3 %** | Isolated-wheel Fluent, `cfd/wheel/` |

The drag result is the main finding. A duct can't save more drag than the
momentum of the air it swallows. Here that is 0.155 kg/s × 83 m/s ≈ 13 N,
about 1 % of the wheel's drag, so no louver design gets close to 18 %.

The full numbers, method and limits are in [docs/RESULTS.md](docs/RESULTS.md),
and the write-up is in [report/ThermoKinetic_BrakeDuct_Whitepaper.pdf](report/ThermoKinetic_BrakeDuct_Whitepaper.pdf).

## Limits (read before quoting numbers)

- ANSYS Student licence: meshes are capped at 1M cells, so the CFD is coarse.
- Steady RANS. The aero model uses a stationary wheel on a slip ground, so the
  absolute drag coefficient (~1.3) is too high. Only the open-vs-closed
  difference is meaningful.
- The lap model is reduced-order, on a synthetic lap trace, calibrated to two
  steady Fluent cases.
- The NiTiCu transformation temperatures are design targets, not a datasheet
  alloy.
- **No hardware was built or tested.** The bench file in `bench/` is a
  simulated dry run of the logging script.
- Moving aero parts are banned under the F1 technical regulations, so this
  is a research and non-FIA concept.

## Tools

SOLIDWORKS 2025, ANSYS Student 2026 R1 (Fluent, MAPDL), Python 3.13
(pywin32, PyFluent, PyMAPDL, gmsh, numpy, matplotlib, ReportLab).
All CAD and simulation runs are driven from the Python scripts in this repo.

## Repo layout

| Path | Contents |
|---|---|
| `phase1_*.py`, `swlib.py` | SOLIDWORKS part and assembly build (COM API) |
| `phase2_*.py` | Fluent CHT domain, meshing, solves, radiation, lap model, wheel aero |
| `phase3_fea.py` | MAPDL linkage and stress model |
| `phase4_bench_validation.py` | DAQ logger (only run in simulation) |
| `phase5_generate_whitepaper.py` | Builds the PDF from the result files |
| `cad/`, `cfd/`, `fea/`, `bench/`, `report/` | Models, plots and result JSON |

Solver case and data files (several GB) are not included. Re-running needs
SOLIDWORKS and ANSYS installed. Run order is in
[docs/RESULTS.md](docs/RESULTS.md#re-running).
