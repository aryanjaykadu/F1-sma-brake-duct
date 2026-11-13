"""Mesh-independence study for the isolated-wheel drag delta: mesh and solve the coarse and medium levels
(open and closed louvers). The fine level is the existing run (no tag). Sizes scale all local and global mesh
sizes together; prism layers (5, first cell 0.1 mm) are kept the same at every level.
Runs one ANSYS session at a time and retries a launch after a pause (Student licence).
"""
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(ROOT, "cfd", "wheel")
PY = sys.executable
LEVELS = {"_medium": 1.35, "_coarse": 1.8}
BASE = {"S_LOUVER": 1.5, "S_DUCT": 4.0, "S_BODY": 12.0, "MAX_SIZE": 100.0}


def run(script, env_extra, log, ok_pattern, tries=4):
    for k in range(tries):
        env = dict(os.environ, **{a: str(b) for a, b in env_extra.items()})
        with open(log, "w") as fh:
            subprocess.run([PY, os.path.join(ROOT, script)], cwd=ROOT, env=env, stdout=fh, stderr=subprocess.STDOUT)
        text = open(log, errors="replace").read()
        if re.search(ok_pattern, text):
            return text
        print(f"  {os.path.basename(log)}: attempt {k + 1} failed, retrying in 90 s", flush=True)
        time.sleep(90)
    raise SystemExit(f"{script} failed: see {log}")


def main():
    jobs = [(v, tag) for tag in LEVELS for v in ("Closed", "Open")]
    for v, tag in jobs:
        mesh = os.path.join(D, f"Wheel_Aero_{v}{tag}.msh.h5")
        log = os.path.join(D, f"mesh_{v}{tag}.log")
        if not (os.path.exists(mesh) and os.path.exists(log) and "cells were created" in open(log, errors="replace").read()):
            env = {k: round(b * LEVELS[tag], 3) for k, b in BASE.items()}
            env.update(MESH_TAG=tag, VARIANT=v)
            text = run("phase2_wheel_mesh.py", env, log, r"MESH WRITTEN")
        else:
            text = open(log, errors="replace").read()
        cells = re.findall(r"(\d+) cells were created", text)[-1]
        print(f"mesh {v}{tag}: {cells} cells", flush=True)
        time.sleep(30)
    for v, tag in jobs:
        out = os.path.join(D, f"aero_{v}{tag}.json")
        if os.path.exists(out) and '"COMPLETED"' in open(out).read():
            print(f"solve {v}{tag}: already done", flush=True)
            continue
        t0 = time.time()
        run("phase2_wheel_solve.py", {"VARIANT": v, "MESH_TAG": tag, "ITER": 600},
            os.path.join(D, f"solve_{v}{tag}.log"), r'"status": "COMPLETED"')
        print(f"solve {v}{tag}: done in {(time.time() - t0) / 60:.0f} min", flush=True)
        time.sleep(60)
    print("STUDY DONE", flush=True)


if __name__ == "__main__":
    main()
