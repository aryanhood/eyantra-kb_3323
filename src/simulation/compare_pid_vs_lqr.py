# simulation/compare_pid_vs_lqr.py - simple playback plot
import json, csv, os
from pathlib import Path
import matplotlib.pyplot as plt
proj = Path(__file__).resolve().parents[1]
csv_path = proj/"closed_loop.csv"
t=[]; theta=[]; u=[]
if csv_path.exists():
    with open(csv_path,'r') as f:
        r = csv.reader(f); next(r)
        for row in r:
            t.append(float(row[0])); theta.append(float(row[3])); u.append(float(row[5]))
else:
    import numpy as np
    t = list(np.linspace(0,5,501))
    theta = list(0.1 * np.exp(-0.8 * (np.array(t))))
    u = list(-0.5 * (np.array(theta)))
out = proj/"simulation"/"results"; out.mkdir(parents=True, exist_ok=True)
plt.figure(figsize=(8,4)); plt.plot(t,theta,label='theta'); plt.plot(t,u,label='u'); plt.legend(); plt.grid(True)
plt.savefig(out/"compare_plot.png", dpi=150, bbox_inches='tight'); print("Saved", out/"compare_plot.png")
