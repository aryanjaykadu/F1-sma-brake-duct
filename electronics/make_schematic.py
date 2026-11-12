"""Generate the shield schematic (KiCad 10) from shield_design.py: symbols in a grid, nets as labels at pins."""
from __future__ import annotations

import os
import uuid

from kicad_lib import load_symbol, pins
from sexp import Sym, dump
from shield_design import PARTS, POWER_NETS, PWR_FLAG_NETS

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kicad")
PROJECT = "sma_rig_shield"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_DNS, "sma-rig-shield"))


def uid():
    return str(uuid.uuid4())


def font():
    return [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]


def prop(name, value, x, y, hide=False):
    p = [Sym("property"), name, value, [Sym("at"), x, y, 0]]
    if hide:
        p.append([Sym("hide"), Sym("yes")])
    p.append(font())
    return p


def snap(v):
    return round(round(v / 1.27) * 1.27, 4)


class Sheet:
    def __init__(self):
        self.lib = {}
        self.items = []
        self.pwr_n = 0

    def place(self, lib_id, ref, value, footprint, x, y):
        sym = load_symbol(lib_id)
        self.lib[lib_id] = sym
        x, y = snap(x), snap(y)
        node = [Sym("symbol"), [Sym("lib_id"), lib_id], [Sym("at"), x, y, 0], [Sym("unit"), 1],
                [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("no" if ref.startswith("#") else "yes")],
                [Sym("on_board"), Sym("no" if ref.startswith("#") else "yes")], [Sym("dnp"), Sym("no")],
                [Sym("uuid"), uid()],
                prop("Reference", ref, x + 2.54, y - 6, hide=ref.startswith("#")),
                prop("Value", value, x + 2.54, y + 6),
                prop("Footprint", footprint, x, y, hide=True),
                prop("Datasheet", "", x, y, hide=True)]
        pmap = pins(sym)
        for num in pmap:
            node.append([Sym("pin"), num, [Sym("uuid"), uid()]])
        node.append([Sym("instances"), [Sym("project"), PROJECT,
                                        [Sym("path"), "/" + ROOT_UUID, [Sym("reference"), ref], [Sym("unit"), 1]]]])
        self.items.append(node)
        # absolute pin endpoints (schematic y is down)
        return {num: (x + px, y - py, ang, name) for num, (name, px, py, ang, _l, _t) in pmap.items()}

    def label(self, net, x, y, pin_angle):
        ang = {0: 180, 180: 0, 90: 270, 270: 90}[int(pin_angle) % 360]
        just = "right" if ang in (180, 270) else "left"
        self.items.append([Sym("label"), net, [Sym("at"), x, y, ang],
                           [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]],
                            [Sym("justify"), Sym(just), Sym("bottom")]], [Sym("uuid"), uid()]])

    def power(self, net, x, y):
        self.pwr_n += 1
        self.place(POWER_NETS.get(net, "power:PWR_FLAG"), f"#PWR{self.pwr_n:03d}", net, "", x, y)

    def flag(self, x, y):
        self.pwr_n += 1
        self.place("power:PWR_FLAG", f"#FLG{self.pwr_n:03d}", "PWR_FLAG", "", x, y)

    def noconnect(self, x, y):
        self.items.append([Sym("no_connect"), [Sym("at"), x, y], [Sym("uuid"), uid()]])

    def text(self, s, x, y):
        self.items.append([Sym("text"), s, [Sym("exclude_from_sim"), Sym("no")], [Sym("at"), x, y, 0],
                           [Sym("effects"), [Sym("font"), [Sym("size"), 2.0, 2.0]], [Sym("justify"), Sym("left"), Sym("bottom")]],
                           [Sym("uuid"), uid()]])

    def write(self, path):
        lib_syms = [Sym("lib_symbols")] + list(self.lib.values())
        doc = [Sym("kicad_sch"), [Sym("version"), Sym("20250610")], [Sym("generator"), "eeschema"],
               [Sym("generator_version"), "10.0"], [Sym("uuid"), ROOT_UUID], [Sym("paper"), "A3"],
               [Sym("title_block"), [Sym("title"), "SMA brake duct bench rig - sensor shield"], [Sym("rev"), "A"]],
               lib_syms] + self.items + [[Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]],
                                         [Sym("embedded_fonts"), Sym("no")]]
        open(path, "w", encoding="utf-8").write(dump(doc) + "\n")


def main():
    os.makedirs(OUT, exist_ok=True)
    sh = Sheet()
    # grid placement (mm on an A3 sheet)
    pos = {"A1": (200, 140)}
    others = [r for r in PARTS if r != "A1"]
    cols = [60, 110, 300, 350]
    for i, ref in enumerate(others):
        c = cols[i % len(cols)]
        r = 50 + (i // len(cols)) * 38
        pos[ref] = (c, r)
    for ref, (lib_id, value, fp, netmap) in PARTS.items():
        x, y = pos[ref]
        endpoints = sh.place(lib_id, ref, value, fp, x, y)
        for num, (px, py, ang, _name) in endpoints.items():
            net = netmap.get(num, "__missing__")
            if net == "__missing__":
                raise SystemExit(f"{ref} pin {num} has no net assignment")
            if net is None:
                sh.noconnect(px, py)
            elif net in POWER_NETS:
                sh.power(net, px, py)
            else:
                sh.label(net, px, py, ang)
    # power flags, each on its own power symbol
    for k, net in enumerate(PWR_FLAG_NETS):
        x, y = 40 + 20 * k, 260
        sh.power(net, x, y)
        sh.flag(x, y)
    sh.text("Logic from USB. 12 V input feeds the fan only. SSR switches the halogen lamp (mains side off-board, "
            "with a KSD301 120 C thermal cut-out in series).", 30, 285)
    path = os.path.join(OUT, PROJECT + ".kicad_sch")
    sh.write(path)
    # minimal project file
    pro = os.path.join(OUT, PROJECT + ".kicad_pro")
    if not os.path.exists(pro):
        open(pro, "w").write('{"meta": {"filename": "%s.kicad_pro", "version": 3}}\n' % PROJECT)
    print("WROTE", path)


if __name__ == "__main__":
    main()
