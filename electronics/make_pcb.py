"""Build the shield PCB (KiCad 10, run with KiCad's python): place footprints, assign nets from the
schematic netlist, route with a 2-layer grid maze router (A*), pour GND on both layers, save.

Router: 0.127 mm grid aligned to the 2.54 mm header pitch, 8-way moves, vias between F.Cu/B.Cu.
Signal tracks 0.3 mm, power (+5V, +12V, VIN_12V, GND) 0.5 mm, clearance 0.2 mm, vias 0.7/0.35 mm.
A cell holds 0 (free), the code of the one net whose copper is within reach, or BLOCK (two nets / edge / hole).
"""
from __future__ import annotations

import heapq
import math
import os
import sys
from array import array

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sexp import find, first, parse  # noqa: E402
from shield_design import PARTS  # noqa: E402

KICAD_FP = os.path.join(os.environ.get("KICAD_DIR", r"C:\Program Files\KiCad\10.0"), "share", "kicad", "footprints")
OUT = os.path.join(HERE, "kicad")
PCB = os.path.join(OUT, "sma_rig_shield.kicad_pcb")
NETLIST = os.path.join(OUT, "sma_rig_shield.net")
OX, OY = 100.0, 100.0                       # board position of Arduino pad 1 (mm)
G = 0.127                                   # grid pitch (mm)
CLR = 0.2
W_SIG, W_PWR = 0.3, 0.5
VIA_D, VIA_DRILL = 0.7, 0.35
POWER = {"+5V", "+12V", "VIN_12V", "GND"}
BLOCK = 0xFFFF

# Uno shield outline (Arduino-local mm, pad 1 = origin)
OUTLINE = [(-28.07, -2.67), (38.23, -2.67), (38.23, 0.0), (40.77, 2.54), (40.77, 35.31), (38.23, 37.85),
           (38.23, 49.28), (36.58, 50.93), (-28.07, 50.93)]
# footprint origin (pad 1) placements, Arduino-local mm, rotation deg
PLACE = {
    "A1": (0, 0, 0),
    "J1": (-10, 4, 0), "J2": (-2, 4, 0), "J3": (6, 4, 0),
    "J4": (14, 4, 0), "J5": (20, 4, 0), "J6": (26, 4, 0), "J7": (32, 9, 0),
    "R1": (-10, 20, 0), "R2": (-10, 23.5, 0), "R3": (-10, 27, 0),
    "R4": (2, 20, 0), "R5": (2, 23.5, 0), "R6": (2, 27, 0),
    "R7": (14, 20, 0), "C1": (14, 23.5, 0), "C2": (14, 27, 0), "C3": (25.2, 20.5, 0),
    "Q1": (24, 27, 0), "D2": (31, 27, 0), "D1": (-10, 33, 0), "SW1": (3.5, 31, 0),
    "J8": (16, 38.5, 0), "J9": (28, 38.5, 0),
}


def mmv(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y))


def read_netlist():
    tree = parse(open(NETLIST, encoding="utf-8").read())
    nets = {}
    for n in find(first(tree, "nets"), "net"):
        name = first(n, "name")[1]
        nodes = [(first(nd, "ref")[1], first(nd, "pin")[1]) for nd in find(n, "node")]
        nets[name] = nodes
    return nets


# ---------------------------------------------------------------- grid
class Grid:
    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = x0, y0
        self.nx = int(math.ceil((x1 - x0) / G)) + 1
        self.ny = int(math.ceil((y1 - y0) / G)) + 1
        self.cells = [array("H", [0]) * (self.nx * self.ny) for _ in range(2)]

    def idx(self, ix, iy):
        return iy * self.nx + ix

    def xy(self, ix, iy):
        return self.x0 + ix * G, self.y0 + iy * G

    def ij(self, x, y):
        return int(round((x - self.x0) / G)), int(round((y - self.y0) / G))

    def mark(self, layer, i, net):
        v = self.cells[layer][i]
        if v == 0:
            self.cells[layer][i] = net
        elif v != net:
            self.cells[layer][i] = BLOCK

    def raster(self, layers, net, bbox, dist_fn):
        x0, y0, x1, y1 = bbox
        i0, j0 = self.ij(x0, y0)
        i1, j1 = self.ij(x1, y1)
        for j in range(max(0, j0 - 1), min(self.ny, j1 + 2)):
            for i in range(max(0, i0 - 1), min(self.nx, i1 + 2)):
                x, y = self.xy(i, j)
                if dist_fn(x, y):
                    k = self.idx(i, j)
                    for L in layers:
                        self.mark(L, k, net)

    def rect(self, layers, net, r, d):
        x0, y0, x1, y1 = r

        def inside(x, y):
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            return dx * dx + dy * dy <= d * d
        self.raster(layers, net, (x0 - d, y0 - d, x1 + d, y1 + d), inside)

    def capsule(self, layers, net, a, b, r):
        ax, ay = a
        bx, by = b
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy or 1e-12

        def inside(x, y):
            t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
            dx, dy = ax + t * vx - x, ay + t * vy - y
            return dx * dx + dy * dy <= r * r
        self.raster(layers, net, (min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r), inside)


def seg_dist(px, py, a, b):
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy or 1e-12
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(ax + t * vx - px, ay + t * vy - py)


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for k in range(n):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


# ---------------------------------------------------------------- build
def build():
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetClearance(pcbnew.FromMM(CLR))
    nc.SetTrackWidth(pcbnew.FromMM(W_SIG))
    nc.SetViaDiameter(pcbnew.FromMM(VIA_D))
    nc.SetViaDrill(pcbnew.FromMM(VIA_DRILL))
    ds.m_MinClearance = pcbnew.FromMM(CLR)
    ds.m_TrackMinWidth = pcbnew.FromMM(0.2)
    ds.m_ViasMinSize = pcbnew.FromMM(0.6)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.3)
    ds.m_CopperEdgeClearance = pcbnew.FromMM(0.3)
    ds.m_HoleClearance = pcbnew.FromMM(0.25)

    netlist = read_netlist()
    netinfo, codes = {}, {}
    for k, name in enumerate(sorted(netlist), start=1):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
        codes[name] = k
    pin_net = {(r, p): n for n, nodes in netlist.items() for r, p in nodes}

    fps = {}
    for ref, (lib_id, value, fpn, _nets) in PARTS.items():
        lib, name = fpn.split(":")
        fp = pcbnew.FootprintLoad(os.path.join(KICAD_FP, lib + ".pretty"), name)
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetFPIDAsString(fpn)
        x, y, rot = PLACE[ref]
        fp.SetPosition(mmv(x, y))
        fp.SetOrientationDegrees(rot)
        if ref == "A1":
            # the shield sits on the Arduino: its courtyard (the whole Uno outline) is not a keep-out here
            for item in list(fp.GraphicalItems()):
                if item.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                    fp.Remove(item)
        board.Add(fp)
        for pad in fp.Pads():
            net = pin_net.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        fps[ref] = fp

    # outline
    for k in range(len(OUTLINE)):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        s.SetStart(mmv(*OUTLINE[k]))
        s.SetEnd(mmv(*OUTLINE[(k + 1) % len(OUTLINE)]))
        board.Add(s)
    t = pcbnew.PCB_TEXT(board)
    t.SetText("SMA DUCT RIG SHIELD  rev A")
    t.SetLayer(pcbnew.F_SilkS)
    t.SetPosition(mmv(-12, 46.0))
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(1.0), pcbnew.FromMM(1.0)))
    board.Add(t)
    return board, fps, netinfo, codes


def pad_rect(pad):
    bb = pad.GetBoundingBox()
    return (pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
            pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY)


def route(board, fps, netinfo, codes, order):
    xs = [p[0] for p in OUTLINE]
    ys = [p[1] for p in OUTLINE]
    grid = Grid(min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
    width = {n: (W_PWR if n in POWER else W_SIG) for n in netinfo}
    edge_keep = 0.3 + W_PWR / 2
    # board edge
    for j in range(grid.ny):
        for i in range(grid.nx):
            x, y = grid.xy(i, j)
            if not point_in_poly(x, y, OUTLINE) or min(
                    seg_dist(x, y, OUTLINE[k], OUTLINE[(k + 1) % len(OUTLINE)]) for k in range(len(OUTLINE))) < edge_keep:
                k = grid.idx(i, j)
                grid.cells[0][k] = BLOCK
                grid.cells[1][k] = BLOCK
    # pads and holes
    pads_by_net = {}
    for ref, fp in fps.items():
        for pad in fp.Pads():
            r = pad_rect(pad)
            net = pad.GetNetname()
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH or not net:
                grid.rect((0, 1), BLOCK, r, CLR + W_PWR / 2 + 0.1)
                continue
            code = codes[net]
            grid.rect((0, 1), code, r, CLR + W_SIG / 2)
            c = pad.GetPosition()
            cx, cy = pcbnew.ToMM(c.x) - OX, pcbnew.ToMM(c.y) - OY
            pads_by_net.setdefault(net, []).append((ref, pad.GetNumber(), r, (cx, cy), pad))

    tracks, vias, failed = [], [], []

    def passable(L, k, code):
        v = grid.cells[L][k]
        return v == 0 or v == code

    def wide_ok(L, i, j, code, rad):
        for dj in range(-rad, rad + 1):
            for di in range(-rad, rad + 1):
                ii, jj = i + di, j + dj
                if not (0 <= ii < grid.nx and 0 <= jj < grid.ny) or not passable(L, grid.idx(ii, jj), code):
                    return False
        return True

    def pad_cells(r, code, pad):
        """Grid cells whose centre lies on the pad copper with 0.15 mm to spare."""
        out = []
        i0, j0 = grid.ij(r[0], r[1])
        i1, j1 = grid.ij(r[2], r[3])
        m = pcbnew.FromMM(0.15)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                x, y = grid.xy(i, j)
                ok = all(pad.HitTest(mmv(x + dx, y + dy), 0) for dx, dy in ((0, 0), (0.15, 0), (-0.15, 0),
                                                                           (0, 0.15), (0, -0.15)))
                if ok:
                    out.append((i, j))
        if not out:
            raise RuntimeError(f"no grid cell on pad {pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}")
        return out

    for net in order:
        if net not in pads_by_net or len(pads_by_net[net]) < 2:
            continue
        code = codes[net]
        w = width[net]
        rad = 1 if w > W_SIG else 0
        pads = pads_by_net[net]
        # merge pads that are the same physical pad repeated (e.g. SW 1/1)
        tree = set()
        cell_pad = {}                        # source cell -> pad centre, to start tracks in the pad middle
        for L in (0, 1):
            for c in pad_cells(pads[0][2], code, pads[0][4]):
                tree.add((L, c[0], c[1]))
                cell_pad[(L, c[0], c[1])] = pads[0][3]
        connected = [pads[0]]
        todo = pads[1:]
        while todo:
            # nearest remaining pad to any connected pad centre
            todo.sort(key=lambda p: min(math.dist(p[3], q[3]) for q in connected))
            tgt = todo.pop(0)
            tcells = set()
            for L in (0, 1):
                for c in pad_cells(tgt[2], code, tgt[4]):
                    tcells.add((L, c[0], c[1]))
            if tree & tcells:
                connected.append(tgt)
                tree |= tcells
                continue
            ti, tj = grid.ij(*tgt[3])
            path = astar(grid, tree, tcells, (ti, tj), code, rad, passable, wide_ok)
            if path is None:
                failed.append((net, tgt[0], tgt[1]))
                continue
            commit_path(grid, path, cell_pad.get(path[0]), tgt, code, w, tracks, vias)
            for node in path:
                tree.add(node)
            tree |= tcells
            for node in tcells:
                cell_pad[node] = tgt[3]
            connected.append(tgt)
    return tracks, vias, failed


def astar(grid, sources, targets, tij, code, rad, passable, wide_ok):
    nx = grid.nx
    ti, tj = tij
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    VIA_COST = 25.0

    def h(i, j):
        dx, dy = abs(i - ti), abs(j - tj)
        return (dx + dy) + (1.4142 - 2) * min(dx, dy)

    openq, best, parent = [], {}, {}
    for s in sources:
        best[s] = 0.0
        heapq.heappush(openq, (h(s[1], s[2]), 0.0, s))
        parent[s] = None
    tset = targets
    expanded = 0
    while openq:
        f, g, node = heapq.heappop(openq)
        if g > best.get(node, 1e18):
            continue
        if node in tset:
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]
        expanded += 1
        if expanded > 900000:
            return None
        L, i, j = node
        for di, dj, c in moves:
            ii, jj = i + di, j + dj
            if not (0 <= ii < nx and 0 <= jj < grid.ny):
                continue
            k = jj * nx + ii
            if not passable(L, k, code):
                continue
            if di and dj and not (passable(L, j * nx + ii, code) and passable(L, jj * nx + i, code)):
                continue
            if rad and not wide_ok(L, ii, jj, code, rad):
                continue
            nn = (L, ii, jj)
            ng = g + c
            if ng < best.get(nn, 1e18):
                best[nn] = ng
                parent[nn] = node
                heapq.heappush(openq, (ng + h(ii, jj), ng, nn))
        # via: other copper must be >= VIA_D/2 + CLR away -> 2 extra free cells on both layers
        o = 1 - L
        if wide_ok(L, i, j, code, 2 + rad) and wide_ok(o, i, j, code, 2 + rad):
            nn = (o, i, j)
            ng = g + VIA_COST
            if ng < best.get(nn, 1e18):
                best[nn] = ng
                parent[nn] = node
                heapq.heappush(openq, (ng + h(i, j), ng, nn))
    return None


def commit_path(grid, path, start_pad, tgt, code, w, tracks, vias):
    # split into same-layer runs, compress collinear points
    runs, cur = [], [path[0]]
    for a, b in zip(path, path[1:]):
        if a[0] != b[0]:
            runs.append(cur)
            vias.append((grid.xy(a[1], a[2]), code))
            grid.capsule((0, 1), code, grid.xy(a[1], a[2]), grid.xy(a[1], a[2]), VIA_D / 2 + CLR + W_SIG / 2)
            cur = [b]
        else:
            cur.append(b)
    runs.append(cur)
    for run in runs:
        L = run[0][0]
        pts = [grid.xy(i, j) for _, i, j in run]
        if run is runs[0] and start_pad is not None:
            pts.insert(0, start_pad)                 # start in the source pad centre
        if run is runs[-1]:
            pts.append(tgt[3])                       # finish in the target pad centre
        simp = [pts[0]]
        for k in range(1, len(pts) - 1):
            ax, ay = simp[-1]
            bx, by = pts[k]
            cx, cy = pts[k + 1]
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) > 1e-9:
                simp.append(pts[k])
        simp.append(pts[-1])
        for a, b in zip(simp, simp[1:]):
            if math.dist(a, b) < 1e-6:
                continue
            tracks.append((L, a, b, w, code))
            grid.capsule((L,), code, a, b, w / 2 + CLR + W_SIG / 2)


def commit_board(board, netinfo, codes, tracks, vias):
    byc = {c: netinfo[n] for n, c in codes.items()}
    for L, a, b, w, code in tracks:
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(mmv(*a))
        t.SetEnd(mmv(*b))
        t.SetWidth(pcbnew.FromMM(w))
        t.SetLayer(pcbnew.F_Cu if L == 0 else pcbnew.B_Cu)
        t.SetNet(byc[code])
        board.Add(t)
    for (x, y), code in vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(mmv(x, y))
        v.SetWidth(pcbnew.FromMM(VIA_D))
        v.SetDrill(pcbnew.FromMM(VIA_DRILL))
        v.SetNet(byc[code])
        board.Add(v)


def add_zones(board, netinfo):
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(netinfo["GND"])
        z.SetLocalClearance(pcbnew.FromMM(0.3))
        z.SetMinThickness(pcbnew.FromMM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)   # GND pads are also track-routed
        ol = z.Outline()
        ol.NewOutline()
        for x, y in OUTLINE:
            ol.Append(mmv(x, y))
        ol.Inflate(-pcbnew.FromMM(0.6), pcbnew.CORNER_STRATEGY_CHAMFER_ALL_CORNERS, pcbnew.FromMM(0.01))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def main():
    netlist = read_netlist()
    signal = sorted([n for n in netlist if n not in POWER], key=lambda n: len(netlist[n]))
    base_order = signal + ["+12V", "VIN_12V", "+5V", "GND"]
    order = list(base_order)
    for attempt in range(5):
        board, fps, netinfo, codes = build()
        tracks, vias, failed = route(board, fps, netinfo, codes, order)
        print(f"attempt {attempt + 1}: {len(tracks)} segments, {len(vias)} vias, failed {failed}")
        if not failed:
            break
        bad = []
        for n, _r, _p in failed:
            if n not in bad:
                bad.append(n)
        order = bad + [n for n in order if n not in bad]
    commit_board(board, netinfo, codes, tracks, vias)
    add_zones(board, netinfo)
    os.makedirs(OUT, exist_ok=True)
    board.Save(PCB)
    print("SAVED", PCB, "failed", failed)


if __name__ == "__main__":
    main()
