"""Minimal S-expression reader/writer for KiCad files."""
from __future__ import annotations

import re

_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))')


class Sym(str):
    """Bare (unquoted) atom."""


def parse(text: str):
    stack, cur, pos = [], [], 0
    n = len(text)
    while pos < n:
        m = _TOKEN.match(text, pos)
        if not m or m.end() == pos:
            if text[pos:].strip() == "":
                break
            raise ValueError(f"bad token at {pos}: {text[pos:pos + 40]!r}")
        pos = m.end()
        if m.group(1):
            stack.append(cur)
            cur = []
        elif m.group(2):
            done = cur
            cur = stack.pop()
            cur.append(done)
        elif m.group(3) is not None:
            cur.append(m.group(3).replace('\\"', '"').replace("\\\\", "\\"))
        else:
            cur.append(Sym(m.group(4)))
    return cur[0] if len(cur) == 1 else cur


def dump(node, indent=0) -> str:
    if isinstance(node, list):
        if not node:
            return "()"
        head = dump(node[0])
        simple = all(not isinstance(c, list) for c in node)
        if simple:
            return "(" + " ".join(dump(c) for c in node) + ")"
        pad = "\t" * (indent + 1)
        parts = [head] + [("\n" + pad + dump(c, indent + 1)) if isinstance(c, list) else (" " + dump(c))
                          for c in node[1:]]
        return "(" + "".join(parts) + "\n" + "\t" * indent + ")"
    if isinstance(node, Sym):
        return str(node)
    if isinstance(node, bool):
        return "yes" if node else "no"
    if isinstance(node, (int, float)):
        s = f"{node:.4f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return '"' + str(node).replace("\\", "\\\\").replace('"', '\\"') + '"'


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def first(node, key):
    r = find(node, key)
    return r[0] if r else None
