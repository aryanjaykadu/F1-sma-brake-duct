"""Phase 7b - animate the SMA duct module opening and closing (SOLIDWORKS frames -> GIF).

Each frame re-poses the louvers (rotation about their shafts), the linkage rod and crosshead (crank kinematics,
R = 10.89 mm) and re-pitches the coil springs to their real length, then saves a PNG. The spring temperature
shown for each frame comes from the MAPDL six-spring sweep (fea/pack_NiTiCu): heating branch while opening,
cooling branch while closing.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import product_cad as pc
from swlib import SW, byref_long, call, dbl_array, get, nothing

ROOT = pc.ROOT
OUT = os.path.join(ROOT, "cad", "product", "anim")
ASM = os.path.join(pc.OUT, "SMA_Duct_Module.SLDASM")
THETA_HOT = 88.48
N = 14


def sweep_tables():
    s = json.load(open(os.path.join(ROOT, "fea", "pack_NiTiCu", "phase3_results.json")))["sweep"]
    heat = sorted((max(st, 0.0), t) for t, st, ok in s["heating"] if ok and 70 <= t <= 76)
    cool = sorted((max(st, 0.0), t) for t, st, ok in s["cooling"] if ok and 58 <= t <= 64)
    return heat, cool


def temp_for(stroke, table):
    xs = [a for a, _ in table]
    ys = [b for _, b in table]
    return float(np.interp(stroke, xs, ys))


def stroke_of(theta):
    return pc.CRANK_R * (math.sin(math.radians(theta) + pc.PHI0) - math.sin(pc.PHI0))


def main():
    os.makedirs(OUT, exist_ok=True)
    heat, cool = sweep_tables()
    sw = SW()
    sw.app.CloseAllDocuments(True)
    err, warn = byref_long(), byref_long()
    doc = sw.app.OpenDoc6(ASM, 2, 1, "", err, warn)
    if doc is None:
        raise SystemExit(f"open failed {err.value}")
    comps = list(doc.GetComponents(True))
    # unfix so transforms can be driven (no mates in this assembly)
    for c in comps:
        doc.ClearSelection2(True)
        call(c, "Select4", False, nothing(), False)
        get(doc, "UnfixComponent")
    doc.ClearSelection2(True)
    by_stem = {}
    for c in comps:
        stem = os.path.splitext(os.path.basename(get(c, "GetPathName")))[0]
        by_stem.setdefault(stem, []).append(c)
    for v in by_stem.values():
        v.sort(key=lambda c: get(c, "Name2"))
    glass = pc.props("carbon")
    glass[7] = 0.6                                 # see-through housing so the louvers are visible
    call(by_stem["Brake_Duct_Housing"][0], "SetMaterialPropertyValues2", dbl_array(glass), 2, nothing())
    springs = {stem: sw.app.GetOpenDocumentByName(os.path.join(pc.OUT, stem + ".SLDPRT"))
               for stem in ("SMA_Spring_NiTiCu_0p5", "Bias_Spring_Steel")}
    coil = {"SMA_Spring_NiTiCu_0p5": (0.5, 19), "Bias_Spring_Steel": (0.8, 14)}

    seq = [("heating", THETA_HOT * k / (N - 1)) for k in range(N)] + [("hot", THETA_HOT)] * 3 + \
          [("cooling", THETA_HOT * (1 - k / (N - 1))) for k in range(N)] + [("cold", 0.0)] * 3
    frames = []
    doc.ShowNamedView2("*Trimetric", -1)
    get(doc, "ViewZoomtofit2")
    for i, (phase, theta) in enumerate(seq):
        sma_len, bias_len = pc.spring_lengths(theta)
        for stem, length in (("SMA_Spring_NiTiCu_0p5", sma_len), ("Bias_Spring_Steel", bias_len)):
            d, turns = coil[stem]
            sdoc = springs[stem]
            prm = call(sdoc, "Parameter", "D4@Coil_Helix")
            call(prm, "SetSystemValue3", (length - d) / turns * pc.MM, 1, nothing())
            sdoc.ForceRebuild3(False)
        seen = {}
        for stem, rot, t in pc.placements(theta):
            k = seen.get(stem, 0)
            seen[stem] = k + 1
            comp = by_stem[stem][k]
            comp.Transform2 = call(sw.math, "CreateTransform",
                                   dbl_array(list(rot) + [v * pc.MM for v in t] + [1.0, 0, 0, 0]))
        doc.ForceRebuild3(False)
        doc.GraphicsRedraw2()
        png = os.path.join(OUT, f"frame_{i:02d}.png")
        e2, w2 = byref_long(), byref_long()
        doc.Extension.SaveAs(png, 0, 1 | 2, nothing(), e2, w2)
        s = stroke_of(theta)
        if phase in ("heating", "hot"):
            T = temp_for(s, heat) if s > 0.01 else 70.0
        else:
            T = temp_for(s, cool) if s > 0.01 else 58.0
        frames.append((png, phase, theta, s, T))
        print(f"frame {i:02d} {phase:8s} theta {theta:5.1f} stroke {s:5.2f} T {T:5.1f}", flush=True)

    # restore the 80 deg pose and fixed components, without saving the drive changes
    sw.app.CloseAllDocuments(True)

    font = ImageFont.truetype("arial.ttf", 30)
    small = ImageFont.truetype("arial.ttf", 22)
    imgs = []
    for png, phase, theta, s, T in frames:
        im = Image.open(png).convert("RGB")
        w, h = im.size
        dr = ImageDraw.Draw(im)
        col = (200, 40, 30) if phase in ("heating", "hot") else (30, 90, 200)
        label = {"heating": "Spring heating", "hot": "Hot: fully open", "cooling": "Spring cooling",
                 "cold": "Cold: closed"}[phase]
        dr.text((30, 25), f"{label}   {T:4.1f} °C", fill=col, font=font)
        dr.text((30, 68), f"Louvers {theta:4.1f}°   rod stroke {s:5.2f} mm", fill=(40, 40, 40), font=small)
        dr.text((30, h - 45), "SMA brake-duct actuator - 6 x 0.5 mm NiTiCu springs, Af 76 °C / Mf 58 °C "
                              "(temperatures from the MAPDL sweep)", fill=(90, 90, 90), font=small)
        bar_w = int((w - 60) * s / 15.35)
        dr.rectangle((30, 105, w - 30, 117), outline=(150, 150, 150))
        dr.rectangle((30, 105, 30 + bar_w, 117), fill=col)
        imgs.append(im)
    gif = os.path.join(ROOT, "cad", "product", "SMA_Duct_Module_actuation.gif")
    durations = [250 if f[1] in ("heating", "cooling") else 600 for f in frames]
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=durations, loop=0, optimize=True)
    imgs[N - 1].save(os.path.join(ROOT, "cad", "product", "Module_open.png"))
    imgs[-1].save(os.path.join(ROOT, "cad", "product", "Module_closed.png"))
    print("GIF", gif, len(imgs), "frames")


if __name__ == "__main__":
    main()
