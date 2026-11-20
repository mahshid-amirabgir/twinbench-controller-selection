"""Link-mass mismatch sweep for one robot at one magnitude (rigid-body tier).

    python experiments/drivers/mismatch_driver.py ROBOT PCT REALIZATIONS SEEDS OUTTAG
    e.g. python experiments/drivers/mismatch_driver.py ur5e 0.10 0,1,2,3,4,5 0,1,2,3,4 ur5e_10

Writes data/d1_mismatch_<OUTTAG>.csv with the columns of data/d1_mismatch_all.csv.
The full sweep is the `mismatch` step of experiments/run_everything.py.
"""
import sys
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import numpy as np, pandas as pd
from experiments import run_main_grid as RE
import twinbench.constants as C

# args: robot  pct(e.g. 0.10)  realizations(comma)  seeds(comma)  outtag
robot   = sys.argv[1]
pct     = float(sys.argv[2])
reals   = [int(x) for x in sys.argv[3].split(',')]
seeds   = [int(x) for x in sys.argv[4].split(',')]
outtag  = sys.argv[5]
TRAJ = list(C.TRAJECTORIES); CTRL = list(C.CONTROLLERS); PARAMS = list(C.PARAMS)

rows=[]; t0=time.time()
for realz in reals:
    RE.PARAM_MISMATCH_FRAC = pct        # module globals read by run_single
    RE.PARAM_MISMATCH_SEED = realz
    for traj in TRAJ:
        for ctrl in CTRL:
            for prm in PARAMS:
                for sd in seeds:
                    res = RE.run_single(robot,'open_reach',traj,ctrl,prm,sd,fidelity='rigid_body')
                    rows.append(dict(robot=robot, pct=pct, realization=realz, trajectory=traj,
                                     controller=ctrl, params=prm, seed=sd,
                                     RMSE_cart_norm=res['RMSE_cart_norm'],
                                     Tau_peak_ratio=res['Tau_peak_ratio'],
                                     Sat_frac_any=res['Sat_frac_any'],
                                     Vel_violation_frac=res['Vel_violation_frac'],
                                     Acc_violation_frac=res['Acc_violation_frac']))
RE.PARAM_MISMATCH_FRAC=None
df=pd.DataFrame(rows)
out=ROOT / "data" / f"d1_mismatch_{outtag}.csv"
df.to_csv(out,index=False)
dt=time.time()-t0
print(f"{out}: {len(df)} runs in {dt:.0f}s ({dt/max(len(df),1):.3f}s/run) | Sat max {df.Sat_frac_any.max()} Acc max {df.Acc_violation_frac.max()}")
