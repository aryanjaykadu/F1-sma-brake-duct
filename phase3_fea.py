"""Phase 3 - SMA actuation and structural stress validation in MAPDL (PyMAPDL), using the
STEP parts exported from SOLIDWORKS in Phase 1.

1. Actuation model (NLGEOM): 4 cranks + rod as BEAM188, pin joints by coupling, SMA and bias
   springs as COMBIN39, louver stops as rotational COMBIN39. Solved cold (25 C), hot (85 C)
   and over a heating/cooling sweep.
2. Pivot model (SOLID187 + frictionless CONTA174/TARGE170): housing + most loaded louver,
   aero pressure on the flap, crank-pin joint force from the actuation model.
3. Rod model (SOLID187, linear): joint forces as cosine bearing loads on the four holes,
   SMA / bias forces on the pegs, 3-2-1 restraint (reactions ~ 0 = equilibrium check).
Mesh sizes are chosen to stay inside the ANSYS Student limit (128k nodes/elements).
"""
from __future__ import annotations

from paths import ROOT as PROJECT_ROOT
import json
import math
import os
import sys

import gmsh
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pyvista as pv
from ansys.mapdl.core import launch_mapdl
from mapdl_archive import save_as_archive

pv.OFF_SCREEN = True
CAD = PROJECT_ROOT + r"\cad"
FEA = PROJECT_ROOT + r"\fea"
RUN = os.path.join(FEA, "mapdl")

# ---------------- geometry (m) ----------------
HX, H_CY, H_X0, PITCH, N = 0.058, 0.140, 0.018, 0.020, 4
L_L, L_W, L_T = 0.056, 0.018, 0.002
SHAFT_R, SHAFT_LEN, WALL_IN, WALL_OUT = 0.0015, 0.005, 0.030, 0.032
CRANK_R = 0.014 / (2 * math.sin(math.radians(40)))
PHI0 = -math.radians(40)
CPIN_R = 0.00125
ROD_L, ROD_W, ROD_T, ROD_HOLE_R, ROD_PEG_R = 0.080, 0.006, 0.002, 0.0013, 0.0008
OFFS = [(k - 1.5) * PITCH for k in range(N)]
# ---------------- materials ----------------
PACF = dict(name="PA-CF", E=8.5e9, nu=0.35, rho=1350.0, Sy=140e6)
AL = dict(name="Al 7075-T6", E=71.7e9, nu=0.33, rho=2810.0, Sy=503e6)
FOS_REQ = 2.5
# ---------------- SMA spring ----------------
SPRING = os.environ.get("SPRING", "baseline")
EA, EM, EPS_L = 75e9, 28e9, 0.065
NU_S = 0.33
TAU_DETWIN, INSTALL_FRAC, HARD = 70e6, 0.6, 0.02
if SPRING == "pack":
    # Six parallel 0.5 mm NiTiCu springs, same spring index (C = 5) and same wire stress as the 1.2 mm
    # baseline, 19 active coils so the recoverable extension stays ~49 mm (from the transient lap study).
    AS, AF, MS, MF = 70.0, 76.0, 64.0, 58.0
    WIRE, COIL, NACT, N_PAR = 0.5e-3, 2.5e-3, 19, 6
    F_REC = N_PAR * 32.0 * (WIRE / 1.2e-3) ** 3 * (6.0e-3 / COIL)
    T_HOT = AF
    FEA_OUT = PROJECT_ROOT + r"\fea\pack_NiTiCu"
else:
    AS, AF, MS, MF = 75.0, 85.0, 55.0, 45.0
    WIRE, COIL, NACT, N_PAR = 1.2e-3, 6.0e-3, 8, 1
    F_REC = 32.0
    T_HOT = 85.0
    FEA_OUT = FEA
BIAS_K, BIAS_PRE = 1200.0, 5.0
STOP_K, OPEN_STOP = 50.0, math.radians(90)
Q_AERO = 3800.0
T_COLD = 25.0


def k_spring(E):
    return N_PAR * E / (2 * (1 + NU_S)) * WIRE ** 4 / (8 * COIL ** 3 * NACT)


K_A, K_M = k_spring(EA), k_spring(EM)
DELTA_L = EPS_L * math.pi * NACT * COIL ** 2 / WIRE
INSTALL = INSTALL_FRAC * DELTA_L
C_IDX = COIL / WIRE
WAHL = (4 * C_IDX - 1) / (4 * C_IDX - 4) + 0.615 / C_IDX
F_DETWIN = N_PAR * TAU_DETWIN * math.pi * WIRE ** 3 / (8 * COIL * WAHL)
RATE = F_REC / (AF - AS)


def aust_curve(T):
    plateau = max(F_REC, RATE * (T - AS))
    dy = plateau / K_A
    kh = HARD * K_A
    fl = plateau + kh * (DELTA_L - dy)
    return [(-0.05, -K_A * 0.05), (0, 0), (dy, plateau), (DELTA_L, fl), (DELTA_L + 0.03, fl + K_A * 0.03)]


def mart_curve():
    dy = F_DETWIN / K_M
    fe = F_DETWIN + HARD * K_M * (0.05 - dy)
    return [(-0.05, -fe), (-dy, -F_DETWIN), (0, 0), (dy, F_DETWIN), (0.05, fe)]


def xi_heat(T):
    return 1.0 if T <= AS else 0.0 if T >= AF else 0.5 * (math.cos(math.pi * (T - AS) / (AF - AS)) + 1)


def xi_cool(T):
    return 0.0 if T >= MS else 1.0 if T <= MF else 0.5 * math.cos(math.pi * (T - MF) / (MS - MF)) + 0.5


def interp(curve, d):
    return float(np.interp(d, [p[0] for p in curve], [p[1] for p in curve]))


# ---------------- aero model ----------------
def aero_cn(alpha):
    a = abs(alpha)
    return max(1.98 * math.sin(a), min(2 * math.pi * math.sin(a), 1.2))


def aero_force(theta):
    return Q_AERO * L_W * L_L * aero_cn(math.pi / 2 - theta)


def aero_moment(theta):
    alpha = math.pi / 2 - theta
    return aero_force(theta) * 0.25 * L_W * (1 - math.sin(abs(alpha)))


def aero_worst(theta_max):
    th = np.linspace(0, theta_max, 1601)
    m = np.array([aero_moment(t) for t in th])
    i = int(m.argmax())
    return float(th[i]), float(m[i]), aero_force(float(th[i]))


def stroke_of(theta):
    return CRANK_R * (math.sin(PHI0 + theta) - math.sin(PHI0))


def required_force(theta, m_aero):
    return BIAS_PRE + BIAS_K * stroke_of(theta) + N * m_aero / (CRANK_R * math.cos(PHI0 + theta))


# =====================================================================================
# 1. ACTUATION MODEL
# =====================================================================================
class Mechanism:
    NP, NC, NR, NG = 100, 200, 300, 400
    N_BOT, N_TOP, N_AA, N_AM, N_AB = 501, 502, 601, 602, 603
    E_CR, E_ROD, E_A, E_M, E_B, E_ST = 1, 11, 21, 22, 23, 31

    def __init__(self, mapdl):
        self.m = mapdl
        self.xy = {}

    def _n(self, n, x, y):
        self.m.n(n, x, y, 0)
        self.xy[n] = (x, y)

    def _curve(self, rset, curve):
        flat = [v for p in curve for v in p]
        self.m.r(rset, *flat[:6])
        for i in range(6, len(flat), 6):
            self.m.rmore(*flat[i:i + 6])

    def build(self):
        m = self.m
        m.clear()
        m.prep7()
        m.et(1, "BEAM188")
        m.et(2, "COMBIN39")
        m.keyopt(2, 4, 1)
        m.et(3, "COMBIN39")
        m.keyopt(3, 3, 6)
        m.mp("EX", 1, 200e9)
        m.mp("PRXY", 1, 0.3)
        m.mp("EX", 2, AL["E"])
        m.mp("PRXY", 2, AL["nu"])
        m.sectype(1, "BEAM", "RECT")
        m.secdata(0.005, 0.002)
        m.sectype(2, "BEAM", "RECT")
        m.secdata(ROD_W, ROD_T)
        ys = []
        for k, off in enumerate(OFFS):
            px, py = HX, H_CY + off
            cx, cy = px + CRANK_R * math.cos(PHI0), py + CRANK_R * math.sin(PHI0)
            self._n(self.NP + k, px, py)
            self._n(self.NC + k, cx, cy)
            self._n(self.NR + k, cx, cy)
            self._n(self.NG + k, px, py)
            ys.append(cy)
        rx, ym = self.xy[self.NR][0], float(np.mean(ys))
        self._n(self.N_BOT, rx, ym - ROD_L / 2)
        self._n(self.N_TOP, rx, ym + ROD_L / 2)
        self._n(self.N_AA, rx, ym + ROD_L / 2 + 0.03)
        self._n(self.N_AM, rx, ym + ROD_L / 2 + 0.03)
        self._n(self.N_AB, rx, ym - ROD_L / 2 - 0.03)
        m.type(1)
        m.mat(1)
        m.secnum(1)
        for k in range(N):
            m.en(self.E_CR + k, self.NP + k, self.NC + k)
        m.mat(2)
        m.secnum(2)
        chain = [self.N_BOT] + [self.NR + k for k in range(N)] + [self.N_TOP]
        for i in range(len(chain) - 1):
            m.en(self.E_ROD + i, chain[i], chain[i + 1])
        self._curve(1, aust_curve(T_HOT))
        self._curve(2, mart_curve())
        self._curve(3, [(-0.2, -BIAS_K * 0.2), (0, 0), (0.2, BIAS_K * 0.2)])
        self._curve(4, [(-0.5, -STOP_K * 0.5), (0, 0), (OPEN_STOP, 1e-5 * OPEN_STOP),
                        (OPEN_STOP + 0.5, 1e-5 * OPEN_STOP + STOP_K * 0.5)])
        m.type(2)
        m.mat(1)
        m.real(1)
        m.en(self.E_A, self.N_TOP, self.N_AA)
        m.real(2)
        m.en(self.E_M, self.N_TOP, self.N_AM)
        m.real(3)
        m.en(self.E_B, self.N_BOT, self.N_AB)
        m.type(3)
        m.real(4)
        for k in range(N):
            m.en(self.E_ST + k, self.NG + k, self.NP + k)
        for k in range(N):
            m.cp(1 + 2 * k, "UX", self.NC + k, self.NR + k)
            m.cp(2 + 2 * k, "UY", self.NC + k, self.NR + k)
        m.finish()

    def solve(self, label, T, xi, m_aero):
        m = self.m
        out = dict(label=label, T=T, xi=xi, converged=False)
        m.prep7()
        self._curve(1, [(d, f * max(1 - xi, 1e-6)) for d, f in aust_curve(T)])
        self._curve(2, [(d, f * max(xi, 1e-6)) for d, f in mart_curve()])
        m.finish()
        m.slashsolu()
        m.antype("STATIC", "NEW")
        m.nlgeom("ON")
        m.nsubst(100, 5000, 20)
        m.autots("ON")
        m.neqit(60)
        m.outres("ALL", "LAST")
        m.ddele("ALL", "ALL")
        m.fdele("ALL", "ALL")
        beams = [self.NP + k for k in range(N)] + [self.NC + k for k in range(N)] + \
                [self.NR + k for k in range(N)] + [self.N_BOT, self.N_TOP]
        for n in beams:
            for dof in ("UZ", "ROTX", "ROTY"):
                m.d(n, dof, 0)
        for k in range(N):
            m.d(self.NP + k, "UX", 0)
            m.d(self.NP + k, "UY", 0)
            m.d(self.NG + k, "ALL", 0)
            m.f(self.NP + k, "MZ", -m_aero)
        for n in (self.N_AA, self.N_AM, self.N_AB):
            m.d(n, "ALL", 0)
        m.d(self.N_AA, "UY", INSTALL)
        m.d(self.N_AB, "UY", -BIAS_PRE / BIAS_K)
        try:
            m.solve()
            out["converged"] = int(round(m.get_value("ACTIVE", 0, "SOLU", "CNVG"))) == 1
        except Exception as exc:  # noqa: BLE001
            out["error"] = str(exc).strip()[:200]
        m.finish()
        if out["converged"]:
            m.post1()
            m.set("LAST")
            out["theta"] = [m.get_value("NODE", self.NP + k, "ROT", "Z") for k in range(N)]
            out["stroke"] = m.get_value("NODE", self.NR, "U", "Y")
            out["lateral"] = m.get_value("NODE", self.NR, "U", "X")
            out["sma"] = self._fsum([self.E_A, self.E_M], self.N_TOP, True)
            out["bias"] = self._fsum([self.E_B], self.N_BOT, True)
            out["joint"] = [self._fsum([self.E_CR + k], self.NC + k) for k in range(N)]
            out["disp"] = {n: (m.get_value("NODE", n, "U", "X"), m.get_value("NODE", n, "U", "Y")) for n in self.xy}
            m.finish()
        th = math.degrees(float(np.mean(out["theta"]))) if out["converged"] else float("nan")
        print(f"  mech {label:12s} T={T:5.1f} xi={xi:.3f} conv={out['converged']} theta={th:6.2f} deg "
              f"stroke={out.get('stroke', float('nan')) * 1e3:6.2f} mm")
        return out

    def _fsum(self, elems, node, negate=False):
        """Force exerted by `elems` on `node` (negate=True -> force exerted on the node by the springs)."""
        m = self.m
        m.esel("NONE")
        for e in elems:
            m.esel("A", "ELEM", "", e)
        m.nsel("S", "NODE", "", node)
        m.fsum()
        fx, fy = m.get_value("FSUM", 0, "ITEM", "FX"), m.get_value("FSUM", 0, "ITEM", "FY")
        m.allsel()
        return (-fx, -fy) if negate else (fx, fy)


# =====================================================================================
# 2. MESHING (gmsh, STEP from SOLIDWORKS)
# =====================================================================================
def mesh_step(stem, h_global, h_local, refine_pred):
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setString("Geometry.OCCTargetUnit", "M")
        gmsh.model.occ.importShapes(os.path.join(CAD, stem + ".STEP"))
        gmsh.model.occ.synchronize()
        refine = []
        for _, tag in gmsh.model.getEntities(2):
            if "ylinder" not in gmsh.model.getType(2, tag):
                continue
            lo, hi = gmsh.model.getParametrizationBounds(2, tag)
            k = gmsh.model.getPrincipalCurvatures(tag, [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])])
            kk = max(abs(k[0][0]), abs(k[1][0]))
            if kk < 1e-9:
                continue
            bb = gmsh.model.getBoundingBox(2, tag)
            ctr = [(bb[i] + bb[i + 3]) / 2 for i in range(3)]
            if refine_pred(1 / kk, ctr):
                refine.append(tag)
        fd = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(fd, "SurfacesList", refine)
        gmsh.model.mesh.field.setNumber(fd, "Sampling", 40)
        ft = gmsh.model.mesh.field.add("Threshold")
        for key, val in (("InField", fd), ("SizeMin", h_local), ("SizeMax", h_global), ("DistMin", h_local),
                         ("DistMax", 0.004)):
            gmsh.model.mesh.field.setNumber(ft, key, val)
        gmsh.model.mesh.field.setAsBackgroundMesh(ft)
        for opt in ("MeshSizeExtendFromBoundary", "MeshSizeFromPoints", "MeshSizeFromCurvature"):
            gmsh.option.setNumber("Mesh." + opt, 0)
        gmsh.option.setNumber("Mesh.MeshSizeMax", h_global)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
        gmsh.option.setNumber("Mesh.SecondOrderLinear", 1 if stem != "Control_Linkage_Rod" else 0)
        gmsh.option.setNumber("Mesh.HighOrderOptimize", 0 if stem != "Control_Linkage_Rod" else 2)
        gmsh.model.mesh.generate(3)
        tags, coords, _ = gmsh.model.mesh.getNodes()
        order = np.argsort(tags)
        tags = np.asarray(tags)[order]
        pts = np.asarray(coords).reshape(-1, 3)[order]
        _, en = gmsh.model.mesh.getElementsByType(11)
        cells = np.searchsorted(tags, np.asarray(en)).reshape(-1, 10)[:, [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]]
        used = np.unique(cells)
        remap = -np.ones(len(pts), dtype=np.int64)
        remap[used] = np.arange(len(used))
        print(f"  gmsh {stem}: {len(used)} nodes, {len(cells)} tets, {len(refine)} refined faces")
        return pts[used], remap[cells]
    finally:
        gmsh.finalize()


def write_cdb(path, parts):
    """parts: list of (points, cells, mat_id)."""
    pts, cells, mats, off = [], [], [], 0
    for p, c, mid in parts:
        pts.append(p)
        cells.append(c + off)
        mats.append(np.full(len(c), mid))
        off += len(p)
    pts, cells, mats = np.vstack(pts), np.vstack(cells), np.concatenate(mats)
    grid = pv.UnstructuredGrid({pv.CellType.QUADRATIC_TETRA: cells.astype(np.int64)}, pts)
    n, e = grid.n_points, grid.n_cells
    grid.point_data["ansys_node_num"] = np.arange(1, n + 1, dtype=np.int32)
    grid.cell_data["ansys_elem_num"] = np.arange(1, e + 1, dtype=np.int32)
    grid.cell_data["ansys_material_type"] = mats.astype(np.int32)
    grid.cell_data["ansys_etype"] = np.ones(e, dtype=np.int32)
    grid.cell_data["ansys_elem_type_num"] = np.full(e, 187, dtype=np.int32)
    grid.cell_data["ansys_real_constant"] = np.ones(e, dtype=np.int32)
    save_as_archive(path, grid)
    return n, e


def load_cdb(m, path):
    m.clear()
    m.prep7()
    m.shpp("WARN")
    m.upload(path)
    name = os.path.basename(path)
    m.cdread("DB", name.rsplit(".", 1)[0], "cdb")
    m.et(1, 187)
    m.allsel()
    return int(m.get_value("NODE", 0, "NUM", "MAX")), int(m.get_value("NODE", 0, "COUNT")), \
        int(m.get_value("ELEM", 0, "COUNT"))


def solid_fields(m, mats=None):
    """Return (grid, seqv, usum, mat-per-node mask helper) for SOLID187 elements."""
    m.allsel()
    m.esel("S", "ENAME", "", 187)
    m.nsle("S")
    grid = m.mesh.grid.copy()
    nnum = np.asarray(m.mesh.nnum)
    seqv = np.asarray(m.post_processing.nodal_eqv_stress())
    usum = np.asarray(m.post_processing.nodal_displacement("NORM"))
    lookup_s = np.zeros(int(nnum.max()) + 1)
    lookup_u = np.zeros(int(nnum.max()) + 1)
    lookup_s[nnum] = seqv
    lookup_u[nnum] = usum
    gn = np.asarray(grid.point_data["ansys_node_num"])
    grid.point_data["seqv_MPa"] = lookup_s[gn] / 1e6
    grid.point_data["usum_mm"] = lookup_u[gn] * 1e3
    m.allsel()
    return grid


def node_mats(grid):
    """Material id per point (first element touching the node)."""
    mat = np.asarray(grid.cell_data["ansys_material_type"])
    out = np.zeros(grid.n_points, dtype=int)
    for mid in np.unique(mat):
        sub = grid.extract_cells(np.where(mat == mid)[0])
        out[np.asarray(sub.point_data["vtkOriginalPointIds"])] = mid
    return out


# =====================================================================================
# 3. PIVOT MODEL (housing + louver, contact)
# =====================================================================================
def run_pivot(m, cdb, label, y_piv, crank_force, p0, slope):
    nmax, nn, ne = load_cdb(m, cdb)
    print(f"  pivot [{label}] nodes={nn} elems={ne}")
    m.mp("EX", 1, PACF["E"])
    m.mp("PRXY", 1, PACF["nu"])
    m.mp("EX", 3, PACF["E"])
    m.mp("PRXY", 3, PACF["nu"])
    m.et(2, "MASS21")
    m.r(2, 1e-9, 1e-9, 1e-9, 1e-15, 1e-15, 1e-15)
    m.et(3, "TARGE170")
    m.et(4, "CONTA174")
    m.keyopt(4, 9, 1)
    m.keyopt(4, 10, 2)
    m.mp("MU", 10, 0.0)
    m.local(11, 1, HX, y_piv, 0)          # cylindrical about the pivot axis (Z)
    tol = 3e-5
    for i, (z0, z1) in enumerate(((WALL_IN - 1e-4, WALL_OUT + 1e-4), (-WALL_OUT - 1e-4, -WALL_IN + 1e-4))):
        rset = 3 + i
        m.r(rset)
        m.rmodif(rset, 3, 10.0)           # FKN: stiff normal contact
        for mat, etype in ((1, 3), (3, 4)):
            m.csys(11)
            m.esel("S", "MAT", "", mat)
            m.nsle("S")
            m.nsel("R", "LOC", "X", SHAFT_R - tol, SHAFT_R + tol)
            m.nsel("R", "LOC", "Z", z0 - (0.002 if mat == 3 else 0), z1 + (0.002 if mat == 3 else 0))
            m.csys(0)
            if int(m.get_value("NODE", 0, "COUNT")) == 0:
                raise RuntimeError(f"no contact nodes (mat {mat}, side {i})")
            m.type(etype)
            m.real(rset)
            m.mat(10)
            m.esurf()
            m.allsel()
    # rigid cap on the -Z shaft end: axial + torque restraint only
    z_end = -(L_L / 2 + SHAFT_LEN)
    pilot = nmax + 1
    m.n(pilot, HX, y_piv, z_end)
    m.type(2)
    m.real(2)
    m.mat(3)
    m.e(pilot)
    m.esel("S", "MAT", "", 3)
    m.nsle("S")
    m.nsel("R", "LOC", "Z", z_end - 1e-6, z_end + 1e-6)
    m.nsel("A", "NODE", "", pilot)
    m.cerig(pilot, "ALL", "ALL")
    m.allsel()
    m.finish()

    m.slashsolu()
    m.antype("STATIC", "NEW")
    m.nsel("S", "LOC", "X", H_X0 - 1e-6, H_X0 + 1e-6)
    m.d("ALL", "ALL", 0)
    m.allsel()
    m.d(pilot, "UZ", 0)
    m.d(pilot, "ROTZ", 0)
    # aero pressure on the upstream flap face (x = HX + t/2)
    m.esel("S", "MAT", "", 3)
    m.nsle("S")
    m.nsel("R", "LOC", "X", HX + L_T / 2 - 1e-6, HX + L_T / 2 + 1e-6)
    m.nsel("R", "LOC", "Z", -L_L / 2 - 1e-6, L_L / 2 + 1e-6)
    if slope:
        m.sfgrad("PRES", 0, "Y", y_piv, slope)
    m.sf("ALL", "PRES", p0)
    m.sfgrad()
    m.allsel()
    # crank-pin joint force, spread over the pin surface nodes
    px, py = HX + CRANK_R * math.cos(PHI0), y_piv + CRANK_R * math.sin(PHI0)
    m.local(12, 1, px, py, 0)
    m.esel("S", "MAT", "", 3)
    m.nsle("S")
    m.nsel("R", "LOC", "X", CPIN_R - tol, CPIN_R + tol)
    m.nsel("R", "LOC", "Z", L_L / 2 + SHAFT_LEN + 0.002 + 1e-5, 1.0)
    m.csys(0)
    npin = int(m.get_value("NODE", 0, "COUNT"))
    if npin == 0:
        raise RuntimeError("crank pin nodes not found")
    m.f("ALL", "FX", crank_force[0] / npin)
    m.f("ALL", "FY", crank_force[1] / npin)
    m.allsel()
    m.nlgeom("OFF")
    m.nsubst(5, 50, 2)
    m.autots("ON")
    m.neqit(40)
    m.outres("ALL", "LAST")
    res = dict(label=label, converged=False, nodes=nn, elements=ne)
    try:
        m.solve()
        res["converged"] = int(round(m.get_value("ACTIVE", 0, "SOLU", "CNVG"))) == 1
    except Exception as exc:  # noqa: BLE001
        res["error"] = str(exc).strip()[:300]
    m.finish()
    if not res["converged"]:
        print("   NOT CONVERGED:", res.get("error"))
        return res, None
    m.post1()
    m.set("LAST")
    grid = solid_fields(m)
    pts, s, mats = grid.points, grid.point_data["seqv_MPa"], node_mats(grid)
    r_piv = np.hypot(pts[:, 0] - HX, pts[:, 1] - y_piv)
    lv, hs = mats == 3, mats == 1
    pin_zone = lv & (np.abs(pts[:, 2]) > L_L / 2 - 1e-4) & (pts[:, 2] > z_end + 0.0015) & (r_piv < 0.004)
    crank_zone = lv & (pts[:, 2] > L_L / 2 + SHAFT_LEN - 1e-5)
    hole_zone = hs & (r_piv < 0.004)
    res["peaks_MPa"] = dict(louver_pins=float(s[pin_zone].max()), crank_arm=float(s[crank_zone].max()),
                            louver_flap=float(s[lv & (np.abs(pts[:, 2]) < L_L / 2 - 1e-4)].max()),
                            housing_holes=float(s[hole_zone].max()))
    res["restraint_torque_Nmm"] = m.get_value("NODE", pilot, "RF", "MZ") * 1e3
    m.finish()
    print("   peaks MPa:", {k: round(v, 3) for k, v in res["peaks_MPa"].items()},
          "| restraint torque N-mm:", round(res["restraint_torque_Nmm"], 3))
    return res, grid.extract_cells(np.where(np.asarray(grid.cell_data["ansys_material_type"]) == 3)[0])


# =====================================================================================
# 4. ROD MODEL (bearing loads)
# =====================================================================================
def run_rod(m, cdb, label, joint_on_rod, sma, bias):
    nmax, nn, ne = load_cdb(m, cdb)
    print(f"  rod [{label}] nodes={nn} elems={ne}")
    m.mp("EX", 2, AL["E"])
    m.mp("PRXY", 2, AL["nu"])
    m.finish()
    nodes = np.asarray(m.mesh.nodes)
    nnum = np.asarray(m.mesh.nnum)
    m.slashsolu()
    m.antype("STATIC", "NEW")

    def apply(mask, fx, fy):
        ids = nnum[mask]
        for nid in ids:
            m.f(int(nid), "FX", fx / len(ids))
            m.f(int(nid), "FY", fy / len(ids))

    with m.non_interactive:
        for k, off in enumerate(OFFS):
            fx, fy = joint_on_rod[k]
            mag = math.hypot(fx, fy)
            if mag < 1e-9:
                continue
            dx, dy = nodes[:, 0], nodes[:, 1] - off
            r = np.hypot(dx, dy)
            on = np.abs(r - ROD_HOLE_R) < 3e-5
            cosang = (dx * fx + dy * fy) / (np.maximum(r, 1e-12) * mag)
            w = np.where(on & (cosang > 0), cosang, 0.0)
            w /= w.sum()
            for nid, wi in zip(nnum[w > 0], w[w > 0]):
                m.f(int(nid), "FX", fx * wi)
                m.f(int(nid), "FY", fy * wi)
        top = nodes[:, 1] > ROD_L / 2 + 1e-5
        bot = nodes[:, 1] < -ROD_L / 2 - 1e-5
        apply(top, *sma)
        apply(bot, *bias)
    m.nsel("S", "LOC", "Z", -1e-7, 1e-7)
    m.d("ALL", "UZ", 0)
    m.allsel()
    mid = int(nnum[np.argmin(np.hypot(nodes[:, 0], nodes[:, 1] - 0.010) + np.abs(nodes[:, 2]))])
    far = int(nnum[np.argmin(np.hypot(nodes[:, 0], nodes[:, 1] + 0.030) + np.abs(nodes[:, 2]))])
    m.d(mid, "UX", 0)
    m.d(mid, "UY", 0)
    m.d(far, "UX", 0)
    m.outres("ALL", "LAST")
    res = dict(label=label, converged=False, nodes=nn, elements=ne)
    try:
        m.solve()
        res["converged"] = True
    except Exception as exc:  # noqa: BLE001
        res["error"] = str(exc).strip()[:300]
    m.finish()
    if not res["converged"]:
        return res, None
    m.post1()
    m.set("LAST")
    grid = solid_fields(m)
    pts, s = grid.points, grid.point_data["seqv_MPa"]
    near = np.zeros(len(pts), dtype=bool)
    for off in OFFS:
        near |= np.hypot(pts[:, 0], pts[:, 1] - off) < 0.0028
    res["peaks_MPa"] = dict(rod_holes=float(s[near].max()), rod_pegs=float(s[np.abs(pts[:, 1]) > ROD_L / 2 - 0.002].max()),
                            rod_overall=float(s.max()))
    rf = [m.get_value("NODE", mid, "RF", "FX"), m.get_value("NODE", mid, "RF", "FY"), m.get_value("NODE", far, "RF", "FX")]
    res["restraint_reactions_N"] = rf
    m.finish()
    print("   peaks MPa:", {k: round(v, 3) for k, v in res["peaks_MPa"].items()}, "| reactions N:",
          [round(v, 4) for v in rf])
    return res, grid


# =====================================================================================
# 5. FIGURES
# =====================================================================================
def fig_stress(lv_grid, rod_grid, lv_res, rod_res, path):
    pl = pv.Plotter(off_screen=True, shape=(1, 2), window_size=(3000, 1500))
    pl.subplot(0, 0)
    pl.add_mesh(lv_grid.linear_copy(), scalars="seqv_MPa", cmap="turbo",
                scalar_bar_args={"title": "Louver von Mises [MPa]"})
    pl.add_text(f"Louver, pivot pins and crank - hot ({T_HOT:g} C)\npeak pins {lv_res['peaks_MPa']['louver_pins']:.2f} MPa",
                font_size=13)
    pl.view_isometric()
    pl.subplot(0, 1)
    pl.add_mesh(rod_grid.linear_copy(), scalars="seqv_MPa", cmap="turbo",
                scalar_bar_args={"title": "Rod von Mises [MPa]"})
    pl.add_text(f"Control rod link holes - hot ({T_HOT:g} C)\npeak holes {rod_res['peaks_MPa']['rod_holes']:.2f} MPa",
                font_size=13)
    pl.view_xy()
    pl.screenshot(path)
    pl.close()


def fig_deformation(mech, cold, hot, sweep, path):
    fig, ax = plt.subplots(figsize=(11, 9), dpi=200)

    def geom(st):
        def pos(n):
            x, y = mech.xy[n]
            if st is None:
                return x * 1e3, y * 1e3
            ux, uy = st["disp"][n]
            return (x + ux) * 1e3, (y + uy) * 1e3
        cr = [(pos(mech.NP + k), pos(mech.NC + k)) for k in range(N)]
        rod = [pos(mech.N_BOT)] + [pos(mech.NR + k) for k in range(N)] + [pos(mech.N_TOP)]
        return cr, rod

    for st in sweep:
        if st["converged"]:
            _, rod = geom(st)
            ax.plot(*zip(*rod), color="0.85", lw=1)
    for st, col, name in ((None, "0.4", "Undeformed (closed)"), (cold, "tab:blue", "Case 1 cold 25 C"),
                          (hot, "tab:red", f"Case 2 hot {T_HOT:g} C")):
        if st is not None and not st["converged"]:
            continue
        cr, rod = geom(st)
        ax.plot(*zip(*rod), color=col, lw=3, label=name)
        for a, b in cr:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=2.5)
    if hot["converged"]:
        ns = list(mech.xy)
        x = np.array([mech.xy[n][0] for n in ns]) * 1e3
        y = np.array([mech.xy[n][1] for n in ns]) * 1e3
        u = np.array([hot["disp"][n][0] for n in ns]) * 1e3
        v = np.array([hot["disp"][n][1] for n in ns]) * 1e3
        q = ax.quiver(x, y, u, v, np.hypot(u, v), angles="xy", scale_units="xy", scale=1, cmap="viridis", width=0.004)
        fig.colorbar(q, ax=ax, label="Total displacement [mm]")
        ax.set_title(f"Actuation deformation map 25 C -> {T_HOT:g} C (MAPDL, NLGEOM)\nstroke {hot['stroke'] * 1e3:.2f} mm, "
                     f"louver angle {math.degrees(np.mean(hot['theta'])):.1f} deg")
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fig_hysteresis(hot, heating, cooling, m_star, path):
    fig, axes = plt.subplots(1, 3, figsize=(20, 6.5), dpi=200)
    ax = axes[0]
    f_up = RATE * (T_HOT - MS)
    d1, kh = f_up / K_A, HARD * K_A
    d2, f_top = d1 + DELTA_L, f_up + kh * DELTA_L
    f_low = max(F_REC, RATE * (T_HOT - AS))
    d4 = f_low / K_A
    d3 = (f_top - K_M * d2 - f_low + kh * d4) / (kh - K_M)
    ax.plot(np.array([0, d1, d2]) * 1e3, [0, f_up, f_top], "r-", lw=2.5, label=f"{T_HOT:g} C loading (A->M)")
    ax.plot(np.array([d2, d3, d4, 0]) * 1e3, [f_top, f_low + kh * (d3 - d4), f_low, 0], "r--", lw=2.5,
            label=f"{T_HOT:g} C unloading (M->A)")
    mc = [p for p in mart_curve() if p[0] >= 0]
    ax.plot([p[0] * 1e3 for p in mc], [p[1] for p in mc], "b-", lw=2, label="25 C martensite")
    if hot["converged"]:
        ax.plot((INSTALL - hot["stroke"]) * 1e3, math.hypot(*hot["sma"]), "ko", ms=9, label="MAPDL hot operating point")
    ax.set(xlabel="SMA spring extension [mm]", ylabel="SMA force [N]", title="Nitinol spring force-displacement")
    ax = axes[1]
    s = np.linspace(0, 0.0155, 156)
    avail = [interp(aust_curve(T_HOT), INSTALL - x) for x in s]
    th = [math.asin(min(1, x / CRANK_R + math.sin(PHI0))) - PHI0 for x in s]
    req = [required_force(t, m_star) for t in th]
    ax.plot(s * 1e3, avail, "r-", lw=2.5, label=f"SMA available @ {T_HOT:g} C")
    ax.plot(s * 1e3, req, "k-", lw=2.5, label="Required (bias + max aero)")
    ax.axvline(14, color="g", ls=":", lw=2, label="14 mm target (80 deg)")
    if hot["converged"]:
        ax.plot(hot["stroke"] * 1e3, math.hypot(*hot["sma"]), "ko", ms=9, label="MAPDL equilibrium")
    ax.set(xlabel="Rod stroke [mm]", ylabel="Force [N]", title="Actuator force budget")
    ax = axes[2]
    for data, style, name in ((heating, "r-o", "Heating"), (cooling, "b-s", "Cooling")):
        pts = [(d["T"], d["stroke"] * 1e3) for d in data if d["converged"]]
        if pts:
            ax.plot(*zip(*pts), style, lw=2, label=name)
    for t, nm in ((AS, "As"), (AF, "Af"), (MS, "Ms"), (MF, "Mf")):
        ax.axvline(t, color="0.6", ls="--", lw=1)
        ax.text(t, 0.3, nm, ha="center")
    ax.set(xlabel="SMA temperature [C]", ylabel="Rod stroke [mm]", title="Thermal actuation hysteresis (MAPDL sweep)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# =====================================================================================
def main():
    os.makedirs(RUN, exist_ok=True)
    os.makedirs(FEA_OUT, exist_ok=True)
    th_star, m_star, fn_star = aero_worst(math.radians(80))
    print(f"SMA k_A={K_A / 1e3:.3f} N/mm k_M={K_M / 1e3:.3f} N/mm deltaL={DELTA_L * 1e3:.1f} mm install={INSTALL * 1e3:.1f} mm "
          f"detwin={F_DETWIN:.2f} N | worst aero torque {m_star * 1e3:.2f} N-mm at {math.degrees(th_star):.1f} deg")
    mapdl = launch_mapdl(nproc=4, run_location=RUN, override=True, loglevel="ERROR")
    out = {"inputs": dict(crank_r_mm=CRANK_R * 1e3, k_A_Nmm=K_A / 1e3, k_M_Nmm=K_M / 1e3, delta_L_mm=DELTA_L * 1e3,
                          install_mm=INSTALL * 1e3, detwin_N=F_DETWIN, aero_torque_Nmm=m_star * 1e3,
                          aero_theta_deg=math.degrees(th_star), required_force_80deg_N=required_force(math.radians(80), m_star))}
    try:
        print("== actuation model")
        mech = Mechanism(mapdl)
        mech.build()
        cold = mech.solve("cold_25C", T_COLD, 1.0, aero_moment(0.0))
        hot = mech.solve(f"hot_{T_HOT:g}C", T_HOT, xi_heat(T_HOT), m_star)
        q = (AF - AS) / 4
        temps = sorted({25, MF, MS, AS - 5, AS, AS + q, AS + 2 * q, AS + 3 * q, AF, AF + 10})
        heating = [mech.solve(f"heat_{t}", t, xi_heat(t), m_star) for t in temps]
        qm = (MS - MF) / 4
        temps_c = sorted({AF + 10, AF, AS, MS + 5, MS, MF + 3 * qm, MF + 2 * qm, MF + qm, MF, 25}, reverse=True)
        cooling = [mech.solve(f"cool_{t}", t, xi_cool(t), m_star) for t in temps_c]
        if not (cold["converged"] and hot["converged"]):
            raise RuntimeError("actuation model did not converge")

        print("== meshing STEP parts (gmsh)")
        kstar = int(np.argmax([math.hypot(*f) for f in hot["joint"]]))
        y_star = H_CY + OFFS[kstar]
        hp, hc = mesh_step("Brake_Duct_Housing", 0.005, 0.0005,
                           lambda r, c: abs(r - SHAFT_R) < 2e-5 and abs(c[1] - y_star) < 0.004)
        lp, lc = mesh_step("Nitinol_Louver_Flap", 0.0015, 0.0005, lambda r, c: r < 0.002)
        rp, rc = mesh_step("Control_Linkage_Rod", 0.001, 0.0005, lambda r, c: r < 0.002)
        lpa = np.column_stack([HX - lp[:, 2], y_star + lp[:, 1], lp[:, 0]])     # Phase 1 assembly pose
        piv_cdb, rod_cdb = os.path.join(FEA_OUT, "pivot.cdb"), os.path.join(FEA_OUT, "rod.cdb")
        print("  pivot cdb:", write_cdb(piv_cdb, [(hp, hc, 1), (lpa, lc, 3)]), "rod cdb:", write_cdb(rod_cdb, [(rp, rc, 2)]))

        def to_louver(f, th):
            c, s = math.cos(th), math.sin(th)
            return c * f[0] + s * f[1], -s * f[0] + c * f[1]

        print("== solid stress models")
        runs = {}
        grids = {}
        for key, st, p0, slope in (("cold", cold, Q_AERO, 0.0),
                                   ("hot", hot, fn_star / (L_W * L_L), 12 * m_star / (L_L * L_W ** 3))):
            runs["pivot_" + key], grids["pivot_" + key] = run_pivot(
                mapdl, piv_cdb, key, y_star, to_louver(st["joint"][kstar], st["theta"][kstar]), p0, slope)
            runs["rod_" + key], grids["rod_" + key] = run_rod(
                mapdl, rod_cdb, key, [(-fx, -fy) for fx, fy in st["joint"]], st["sma"], st["bias"])
    finally:
        mapdl.exit()

    print("== figures")
    if grids.get("pivot_hot") is not None and grids.get("rod_hot") is not None:
        fig_stress(grids["pivot_hot"], grids["rod_hot"], runs["pivot_hot"], runs["rod_hot"],
                   os.path.join(FEA_OUT, "FEA_Stress_Contour_Hot.png"))
    fig_deformation(mech, cold, hot, heating, os.path.join(FEA_OUT, "FEA_Deformation_Map.png"))
    fig_hysteresis(hot, heating, cooling, m_star, os.path.join(FEA_OUT, "Nitinol_Force_Displacement_Plot.png"))

    checks = []
    for key, case in (("cold", "Case 1 (25 C)"), ("hot", f"Case 2 ({T_HOT:g} C)")):
        for run, loc, peak_key, mat in (("pivot_", "louver pivot pins", "louver_pins", PACF),
                                        ("pivot_", "louver crank arm + pin", "crank_arm", PACF),
                                        ("pivot_", "housing pivot holes", "housing_holes", PACF),
                                        ("rod_", "rod link holes", "rod_holes", AL)):
            r = runs.get(run + key, {})
            if r.get("converged"):
                peak = r["peaks_MPa"][peak_key]
                fos = mat["Sy"] / 1e6 / peak
                checks.append(dict(location=f"{case} {loc}", material=mat["name"], peak_MPa=peak,
                                   yield_MPa=mat["Sy"] / 1e6, fos=fos, check="PASS" if fos >= FOS_REQ else "FAIL"))
            else:
                checks.append(dict(location=f"{case} {loc}", material=mat["name"], peak_MPa=None,
                                   yield_MPa=mat["Sy"] / 1e6, fos=None, check="NOT CONVERGED"))
    th_hot = math.degrees(float(np.mean(hot["theta"])))
    out.update(stress_checks=checks, runs=runs,
               actuation=dict(cold_theta_deg=math.degrees(float(np.mean(cold["theta"]))), hot_theta_deg=th_hot,
                              hot_stroke_mm=hot["stroke"] * 1e3, hot_sma_force_N=math.hypot(*hot["sma"]),
                              hot_bias_force_N=math.hypot(*hot["bias"]), sma_force_at_Af_N=F_REC,
                              stroke_target_mm=14.0, stroke_check="PASS" if hot["stroke"] >= 0.014 - 1e-6 else "FAIL",
                              angle_check="PASS" if th_hot >= 80 - 1e-3 else "FAIL"),
               sweep=dict(heating=[(d["T"], d.get("stroke", float("nan")) * 1e3, d["converged"]) for d in heating],
                          cooling=[(d["T"], d.get("stroke", float("nan")) * 1e3, d["converged"]) for d in cooling]))
    for k in ("cold", "hot"):
        for d in (cold, hot):
            d.pop("disp", None)
    with open(os.path.join(FEA_OUT, "phase3_results.json"), "w") as fh:
        json.dump(out, fh, indent=2, default=float)
    print("\nSTRESS CHECKS")
    for c in checks:
        pk = f"{c['peak_MPa']:.3f}" if c["peak_MPa"] is not None else "-"
        fs = f"{c['fos']:.1f}" if c["fos"] is not None else "-"
        print(f"  {c['location']:42s} {c['material']:11s} peak {pk:>8s} MPa  FoS {fs:>8s}  {c['check']}")
    print("ACTUATION", json.dumps(out["actuation"], indent=1, default=float))


if __name__ == "__main__":
    sys.exit(main())
