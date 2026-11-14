# F1 Thermo-Kinetic Adaptive Brake Duct

A passive brake-duct louver system driven by a Nitinol shape-memory spring.
Everything here was built and solved on this PC with SOLIDWORKS 2025 SP05 and
ANSYS Student 2026 R1 (Fluent and MAPDL), driven from Python.

The full walkthrough is in the Project Guidebook: `report/Project_Guidebook.pdf`
(source `docs/GUIDEBOOK.md`, built by `make_guidebook.py`).

## Results at a glance

| Phase | Tool | Outcome |
|---|---|---|
| 1. CAD | SOLIDWORKS 2025 | 4 parts + assembly, 16 mates solved. Louver at 80 deg moves the rod 14.000 mm. |
| 2. CFD, Case A (braking) | Fluent 2026 R1 | Pocket air: 300.3 K beside the duct, 324.1 K on the rotor's wake face (492,211 cells): both below 358 K. With rotor radiation the spring wire reaches 378-393 K at emissivity 0.6: **opens**. |
| 2. CFD, drag open vs closed | Fluent 2026 R1 | Isolated wheel at 300 km/h. Stationary wheel on three meshes: the difference sits inside solver noise, and physics caps any saving at 1.1-1.4 %. Rotating wheel on a moving road, time-averaged: closing the louvers **adds 0.53 %** drag (95 % interval +0.43 to +0.63 %). The -18.4 % target is **ruled out**. |
| 2. CFD, Case B (straight) | Fluent 2026 R1 | 300 km/h, 400 C rotor, louvers open (515,700 cells). Wire settles at 306-311 K for any emissivity, below Mf 318 K: **closes**. |
| 3. Structure | MAPDL 2026 R1 | Original spring: 86.2 deg / 14.85 mm at 85 C. Recommended six-spring NiTiCu pack: 88.5 deg / 15.15 mm at 76 C. All stress checks pass FoS 2.5 in both (lowest 3.3). |
| 4. Bench | Python DAQ logger | Logger verified at 20 Hz on a simulated signal only. No DAQ hardware was connected. |
| 5. Report | ReportLab | 6-page whitepaper PDF built from the files above. |
| 6. Bench rig (virtual) | SOLIDWORKS, KiCad 10, arduino-cli, MSVC | Rig CAD (0 interferences), Arduino shield (ERC/DRC clean, Gerbers), firmware (57 % flash, 55 % RAM) and a digital twin running the same firmware code. Twin: 11.5 °C hysteresis on a slow ramp, thin wire closes 3x faster, rig reproduces the car lap's 3 openings (87 % agreement). Not built yet. See `docs/RIG.md`. |
| 7. Product package | SOLIDWORKS, KiCad, ReportLab | Production-detail module CAD with real coil springs (0 interferences), actuation animation, A3 drawing SMA-DM-001 with parts list, 1-page product sheet, rig with the shield PCB in 3D. See `docs/PRODUCT.md`. |

## What the results mean

* **The mechanism works mechanically.** The linkage, spring force budget and
  stresses all check out.
* **Air temperature alone will not trigger the spring.** Beside the duct the
  pocket air stayed at 300 K. Moved to the rotor's outboard wake face it
  reached 324 K, still short of 358 K: the air is hot only in a ~2 mm layer
  against the rotor. The baseline files are kept in `cfd/` with a `_baseline`
  suffix.
* **Radiation can trigger it, if the spring is dark.** Fluent's DO model puts
  119-129 kW/m2 of incident radiation on the relocated pocket. A heat balance
  on the 1.2 mm wire (`phase2_wire_balance.py`, plot
  `cfd/SMA_Wire_Temperature_CaseA.png`) gives:

  | Wire emissivity | Wire temperature 4-15 mm from the rotor | Reaches Af (358 K)? |
  |---|---|---|
  | 0.2 (bright) | 326-331 K | No |
  | 0.4 | 352-360 K | Marginal |
  | 0.6 (oxidised) | 378-387 K | Yes |
  | 0.8 (black coating) | 403-414 K | Yes |

  Needed: emissivity of at least about 0.45, or 0.65 if the coil turns shade
  each other by 30 %.
* **It also recloses on the straight (Case B).** At 300 km/h with a 400 C
  rotor and the louvers open, incident radiation drops to about 20 kW/m2 and
  the airflow doubles. The wire settles at 306-311 K for any emissivity, below
  Mf (318 K), so the bias spring pulls the louvers shut
  (`cfd/SMA_Wire_Temperature_CaseB.png`). On paper the full open/close cycle
  works for an oxidised spring 3-15 mm from the rotor face.
* **The transient lap model overturns the steady verdict for the original
  spring.** `phase2_transient_lap.py` runs 3 laps of a synthetic 98.5 s lap
  (5 braking zones, 300 to 100 km/h). The rotor's heat balance is calibrated
  to the brief: 800 C peak, 400 C on the longest straight. The spring's
  radiation and airflow are fitted to the two Fluent cases, and stroke comes
  from the MAPDL sweep. Results (`cfd/transient/`):

  | Spring | Wire over a lap | Louvers |
  |---|---|---|
  | Original NiTi, 1.2 mm, emissivity 0.6, 6 mm gap | 314-349 K | Never open |
  | Original NiTi, 1.2 mm, emissivity 0.8 | 324-362 K | Open, then stay open 91 % of the time on straights |
  | **NiTiCu, 0.5 mm, Af 76 / As 70 / Ms 64 / Mf 58 C, emissivity 0.8, 3 mm gap** | 313-356 K | **Open fully 4-5 s after the 3 hardest braking zones; open only 2 % of the time on straights** |

  Why: the 1.2 mm wire takes about 5 s to respond while the rotor peak lasts
  only a moment, and NiTi's ~40 K hysteresis is wider than the wire's
  lap-scale swing, so it either never opens or never closes. A thinner wire
  (time constant ~1.4 s) with a narrow-hysteresis alloy fixes both. Catch: at
  the same wire stress a 0.5 mm spring gives about 5.6 N, so about six
  springs in parallel are needed for 32 N.
* **MAPDL re-run for the six-spring pack** (`SPRING=pack python phase3_fea.py`,
  results in `fea/pack_NiTiCu/`). The pack is six 0.5 mm NiTiCu springs, each
  with 2.5 mm coils and 19 active turns, at the same 371 MPa wire stress. It
  delivers 33.3 N and has 48.5 mm of recoverable extension.

  | | Original 1.2 mm NiTi | Six-spring 0.5 mm NiTiCu pack |
  |---|---|---|
  | Hot state | 86.2 deg / 14.85 mm at 85 C | 88.5 deg / 15.15 mm at 76 C |
  | Opens (heating) | 77.5-85 C | 71.5-76 C |
  | Closes (cooling) | 55-47.5 C | 64-58 C |
  | Louver pins, hot | 17.2 MPa, FoS 8.1 | 18.0 MPa, FoS 7.8 |
  | Rod holes, hot | 9.2 MPa, FoS 54.8 | 10.3 MPa, FoS 48.8 |
  | Lowest FoS | 3.3 (housing holes, cold) | 3.3 (housing holes, cold) |

  All stress checks pass. The pack is the recommended actuator.
* **Isolated-wheel drag: any saving is capped at about 1 %, not 18.4 %.**
  `phase2_wheel_cad.py` builds a 2.0 x 1.36 x 3.7 m air box in SOLIDWORKS
  around a 720 mm wheel with an outboard cover, the rotor, and the duct with
  a new forward-facing inlet scoop. `phase2_wheel_mesh.py` meshes it
  (765,201 cells closed, 769,990 open), and `phase2_wheel_solve.py` runs
  Fluent at 300 km/h with k-omega SST for 600 iterations. Results are in
  `cfd/wheel/wheel_aero_open_vs_closed.json`:

  | | Open | Closed | Change |
  |---|---|---|---|
  | Wheel (tyre + rim) drag | 1,144 N | 1,125 N | -19 N |
  | Duct drag | 37.4 N | 38.5 N | +1.0 N |
  | Louver drag | 0.06 N | -0.01 N | ~0 |
  | Total drag, mean of last 200 iterations | 1,183 N | 1,168 N | **-1.3 %** |
  | Iteration oscillation (half-band) | +/-6 N | +/-13 N | |
  | Throat mass flow | 0.155 kg/s | 0.130 kg/s | -16 % |
  | Cd (wheel frontal area 0.212 m2) | 1.31 | 1.29 | |

  The -1.3 % is the same size as the solver's oscillation, and most of it
  shows up on the wheel itself (wake noise), not on the duct or louvers.

  **Mesh-independence study** (phase2_wheel_mesh_study.py,
  phase2_wheel_mesh_compare.py, cfd/wheel/wheel_mesh_independence.json,
  cfd/wheel/Wheel_Mesh_Independence.png). Coarse and medium meshes scale
  every local and global size by 1.8 and 1.35, keeping the same 5 prism
  layers (the 1M-cell Student limit rules out a finer level):

  | Mesh | Cells | Drag open | Drag closed | Change | Solver noise | Momentum limit |
  |---|---|---|---|---|---|---|
  | Coarse | 284k | 1,247 N | 1,128 N | -9.6 % | +/-7.4 % | 1.1 % |
  | Medium | 408k | 1,130 N | 1,189 N | +5.2 % | +/-5.4 % | 1.4 % |
  | Fine | 768k | 1,183 N | 1,168 N | -1.3 % | +/-1.2 % | 1.1 % |

  * **The difference itself is not resolved.** It changes sign between meshes
    and stays inside the iteration noise at every level, so the grid
    convergence index is meaningless for it. Duct drag changes by only 1-2 N
    between open and closed on every mesh. The total swings come from the
    wheel's unsteady wake (-19 N, +41 N, -133 N), which steady RANS cannot
    settle.
  * **The bound is robust.** Even a fully sealed duct can save at most its
    momentum drag, the duct mass flow times speed: 12.9-15.3 N, or 1.1-1.4 %
    of total drag on all three meshes. The -18.4 % target is more than ten
    times this limit.
  * **Resolving the actual difference** would need time-averaged transient
    CFD (URANS or DES), a rotating wheel on a moving ground and a finer mesh
    than the Student licence allows.

  **Rotating wheel and moving road** (phase2_wheel_solve_rot.py,
  phase2_wheel_rot_compare.py, cfd/wheel/wheel_rotating_open_vs_closed.json,
  cfd/wheel/Wheel_Rotating_Transient_Drag.png). Wheel, cover and disc spin at
  231.5 rad/s, the ground moves at 83.3 m/s, fine mesh. 400 steady iterations,
  then 220 transient steps of 1 ms (6 iterations each); the first 60 ms are
  discarded and drag is averaged over 160 ms. The standard error uses the
  integrated autocorrelation time, cross-checked by batch means.

  | | Open | Closed |
  |---|---|---|
  | Steady drag | 1,071.8 N | 1,076.0 N |
  | Time-averaged drag | 1,073.5 +/- 0.2 N | 1,079.2 +/- 0.5 N |
  | Change on closing | | +0.53 % (95 % interval +0.43 to +0.63 %) |
  | Duct drag | 33.3 N | 32.9 N |

  Closing the louvers adds drag in this model: the duct's own drag falls by
  0.4 N, but the wheel's rises by more. Spinning the wheel lowered Cd from
  about 1.3 to 1.19. Caveats: four wake cycles, one mesh, sampling noise
  only.

  The absolute Cd (1.25-1.38 across the meshes, against ~0.6-0.8 typical for
  an isolated wheel) is high: the wheel does not spin, the ground does not
  move, and the mesh is capped at 1M cells.

* **Earlier duct-box drag check (superseded).** `phase2_aero.py` compares louvers
  open (Case B) and closed (`CaseB_closed`) at 300 km/h with a 400 C rotor
  (`cfd/phase2_aero_open_vs_closed.json`):

  | | Open | Closed | Change |
  |---|---|---|---|
  | Louver drag | 3.4 N | 95.0 N | +91.6 N |
  | Duct wall drag | -14.9 N | -8.6 N | +6.3 N |
  | Rotor drag | 753.4 N | 640.7 N | -112.7 N |
  | Total drag in domain | 741.9 N | 727.1 N | -2.0 % |
  | Duct throat mass flow | 0.398 kg/s | 0.292 kg/s | -27 % |
  | Rotor heat removed | 2,172 W | 2,895 W | +33 % |

  The absolute drag and rotor-heat numbers are not usable. The rotor segment
  spans the full width of the 200 x 140 mm CFD box and blocks about 55 % of
  it, so the box behaves like a narrow wind tunnel. Rotor drag comes out
  about 9x what a disc segment should see in open air, and the louver loads
  are inflated too. The louver-load inflation reaches 20-28 N per closed
  louver against the ~4 N used in the stress checks, so the stress checks
  are only valid if the real duct sees no such pressure difference. The
  result that does hold is that closed louvers reduce throat flow by only
  27 %: they leave a 28 % open strip around the louver bank. Verifying the
  -18.4 % target needs an isolated-wheel domain, larger than the Student
  1M-cell limit allows at this mesh resolution. The closed case also
  converged less well (continuity ~3e-2, oscillating), which points to
  unsteady shedding behind the closed louvers. This is an estimate: it neglects heat lost by conduction
  into the spring's end fittings, so real wire temperatures will be lower,
  and the actual emissivity of the chosen wire needs measuring.
* **Regulations.** F1 rules require aerodynamic bodywork to be rigidly secured
  and immobile, however it is actuated. Treat the concept as non-compliant for
  FIA competition unless the FIA confirms otherwise.

## Limits of this work

* CFD convergence is partial: energy and radiation reached about 1e-6, but
  continuity stalled near 9e-3 (target 1e-5). Moving the outlet from 34 mm to
  134 mm behind the rotor cut reversed outlet flow from 59 % to 47 % of the
  area, but did not remove it.
* The CFD geometry is simplified (no pins, pegs or crank arms) and the mesh is
  sized to the ANSYS Student limit. Louver faces are 1 mm, not 0.5 mm.
* The transient lap is a reduced-order model, not a transient Fluent run.
  The lap trace is synthetic (not from a real circuit), the rotor model has
  two fitted constants, the airflow near the spring is interpolated between
  the two Fluent cases, and the NiTiCu transformation temperatures are design
  targets, not a datasheet alloy.
* Case B models the open louvers at 90 deg (MAPDL predicts 86 deg) and uses
  the 400 C straight-line rotor temperature from the brief.
* SMA spring geometry (1.2 mm wire, 6 mm coil, 8 coils) and the aerodynamic
  torque are assumed values, not measured ones.
* The hot-case stress model keeps the louver in its closed pose and rotates
  the loads into it.
* The bench data file is SIMULATED and is labelled as such everywhere.
* The bench rig exists only as CAD, a PCB design, compiled firmware and a
  digital twin. Its results are predictions. The lamp irradiance, blower
  curve and the twin's heat-transfer assumptions must be calibrated on real
  hardware.
* The -18.4 % drag figure in the brief is not reproduced. A stationary wheel
  cannot resolve the open-vs-closed difference (it sits inside the wake noise
  on all three meshes). With a rotating wheel and time averaging, closing
  adds 0.5 %, but that is from 160 ms on one mesh.

## Folder layout

| Folder | Contents |
|---|---|
| `cad/` | SOLIDWORKS parts, assembly, STEP exports, renders |
| `cfd/` | Flow-domain part and Parasolid, Fluent mesh, Case A case/data, contours, results JSON |
| `fea/` | MAPDL meshes, stress and hysteresis figures, results JSON |
| `bench/` | Simulated DAQ log, dashboard, correlation summary |
| `report/` | Whitepaper PDF and its figures |

## Re-running

Use the Python environment with pywin32, ansys-fluent-core, ansys-mapdl-core,
gmsh, mapdl-archive, pyvista, matplotlib, reportlab and pyserial installed.
Run the scripts in this order, one at a time (the Student licence does not
like two ANSYS sessions starting together):

1. `phase1_cad.py`, then `phase1_assembly.py` (SOLIDWORKS must be installed)
2. `phase2_domain_cad.py`, `phase2_mesh.py`, `phase2_solve.py`, `phase2_post.py`
   (Case A as is; Case B with `LOUVERS=open SUFFIX=_CaseB TAG=CaseB CASE_KEY=B`),
   then `phase2_radiation_extract.py` and `phase2_wire_balance.py` per case
   (`TAG=CaseA` or `CaseB`), then `phase2_transient_lap.py` (spring options via
   `ALLOY`, `AS_C`, `AF_C`, `MS_C`, `MF_C`, `WIRE_MM`, `GAP_MM`, `EPS`),
   then for drag `phase2_wheel_cad.py` (`LOUVERS=closed|open`),
   `phase2_wheel_mesh.py` and `phase2_wheel_solve.py` (`VARIANT=Closed|Open`),
   and `phase2_wheel_compare.py`
3. `phase3_fea.py`
4. `phase4_bench_validation.py --port COMx` with a real DAQ board, or
   `--simulate --no-live` for a dry run
5. `phase5_generate_whitepaper.py`
6. Bench rig: `rig_cad.py` (SOLIDWORKS), `electronics/make_schematic.py` then
   `electronics/make_pcb.py` (KiCad's Python), `firmware/gen_lap_profile.py`,
   compile `firmware/sma_rig` with arduino-cli, build `firmware/host/rig_core.dll`
   with MSVC, then `firmware/twin/rig_campaign.py`. Details in `docs/RIG.md`.

The scripts read and write inside the repo folder (override with the `F1_ROOT` environment variable). KiCad scripts look for KiCad in `KICAD_DIR` (default `C:\Program Files\KiCad\10.0`).
