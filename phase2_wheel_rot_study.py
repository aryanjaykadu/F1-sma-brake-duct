"""Run the rotating-wheel / moving-ground cases (fine mesh): Closed, then Open. One ANSYS session at a time."""
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(ROOT, "cfd", "wheel")
for v in ("Closed", "Open"):
    log = os.path.join(D, f"solve_{v}_rot.log")
    for attempt in range(3):
        env = dict(os.environ, VARIANT=v, MESH_TAG="", STEADY_ITER="400", N_STEPS="220", ITER_PER_STEP="6", DT="1e-3")
        t0 = time.time()
        with open(log, "w") as fh:
            subprocess.run([sys.executable, os.path.join(ROOT, "phase2_wheel_solve_rot.py")], cwd=ROOT, env=env,
                           stdout=fh, stderr=subprocess.STDOUT)
        text = open(log, errors="replace").read()
        if '"status": "COMPLETED"' in text:
            print(f"{v}: done in {(time.time() - t0) / 60:.0f} min", flush=True)
            break
        print(f"{v}: attempt {attempt + 1} failed, retrying in 90 s", flush=True)
        time.sleep(90)
    time.sleep(60)
print("ROT STUDY DONE", flush=True)
