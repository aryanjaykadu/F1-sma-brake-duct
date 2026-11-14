"""Run the EV shutter CFD matrix (open / closed at 100 and 130 km/h), one ANSYS session at a time."""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for speed in ("100", "130"):
    for sh in ("open", "closed"):
        tag = f"{'Open' if sh == 'open' else 'Closed'}_{speed}"
        out = os.path.join(HERE, "cfd", f"ev_aero_{tag}.json")
        log = os.path.join(HERE, "cfd", f"solve_{tag}.log")
        for attempt in range(3):
            env = dict(os.environ, SHUTTER=sh, SPEED_KMH=speed, ITER="900")
            t0 = time.time()
            with open(log, "w") as fh:
                subprocess.run([sys.executable, os.path.join(HERE, "ev_solve.py")], cwd=HERE, env=env, stdout=fh,
                               stderr=subprocess.STDOUT)
            if os.path.exists(out) and '"COMPLETED"' in open(out).read() and os.path.getmtime(out) > t0:
                print(f"{tag}: done in {(time.time() - t0) / 60:.0f} min", flush=True)
                break
            print(f"{tag}: attempt {attempt + 1} failed", flush=True)
            time.sleep(90)
        time.sleep(45)
print("EV AERO DONE", flush=True)
