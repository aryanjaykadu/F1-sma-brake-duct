"""Load KiCad library symbols (flattening `extends`) and report their pins."""
from __future__ import annotations

import copy
import os

from sexp import Sym, find, first, parse

KICAD = os.environ.get("KICAD_DIR", r"C:\Program Files\KiCad\10.0")
SYMDIR = os.path.join(KICAD, "share", "kicad", "symbols")
FPDIR = os.path.join(KICAD, "share", "kicad", "footprints")
_cache: dict = {}


def _lib(libname):
    if libname not in _cache:
        tree = parse(open(os.path.join(SYMDIR, libname + ".kicad_sym"), encoding="utf-8").read())
        _cache[libname] = {c[1]: c for c in find(tree, "symbol")}
    return _cache[libname]


def load_symbol(lib_id):
    """Return the symbol S-expression named 'Lib:Name' with any `extends` resolved."""
    libname, name = lib_id.split(":")
    syms = _lib(libname)
    sym = copy.deepcopy(syms[name])
    ext = first(sym, "extends")
    if ext is not None:
        parent = copy.deepcopy(syms[ext[1]])
        # take parent graphics/pins (sub-symbols) renamed to the child, child's properties win
        child_props = {p[1]: p for p in find(sym, "property")}
        out = [Sym("symbol"), name]
        for c in parent[2:]:
            if isinstance(c, list) and c and c[0] == "property" and c[1] in child_props:
                out.append(child_props.pop(c[1]))
            elif isinstance(c, list) and c and c[0] == "symbol":
                c = copy.deepcopy(c)
                c[1] = c[1].replace(ext[1], name, 1)
                out.append(c)
            else:
                out.append(c)
        out.extend(child_props.values())
        sym = out
    sym[1] = lib_id
    return sym


def pins(sym, unit=1):
    """{pin_number: (name, x, y, angle_deg, length, electrical_type)} in library coordinates (y up)."""
    out = {}
    for sub in find(sym, "symbol"):
        parts = sub[1].rsplit("_", 2)
        u = int(parts[-2])
        if u not in (0, unit):
            continue
        for p in find(sub, "pin"):
            at = first(p, "at")
            num = first(p, "number")[1]
            nm = first(p, "name")[1]
            ln = first(p, "length")[1]
            out[num] = (nm, float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0, float(ln), str(p[1]))
    return out


if __name__ == "__main__":
    import sys

    for lid in sys.argv[1:]:
        s = load_symbol(lid)
        print(lid)
        for k, v in sorted(pins(s).items(), key=lambda kv: (kv[1][0])):
            print("  ", k, v)
