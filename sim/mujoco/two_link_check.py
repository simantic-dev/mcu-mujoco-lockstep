# No firmware: (1) the free two-link arm (no damping, no friction) conserves energy; (2) with backlash, each joint's gap is used as in lash_check.py.
import sys, os, numpy as np, mujoco
sys.path.insert(0, os.path.dirname(__file__))
import two_link_xml as X
m = mujoco.MjModel.from_xml_string(X.make(damping=0.0)); d = mujoco.MjData(m)
d.qpos[:] = [1.0, -0.6]; mujoco.mj_forward(m, d)
m.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_ENERGY)
def energy():
    mujoco.mj_forward(m, d); return float(d.energy[0] + d.energy[1])
e0 = energy(); emax = 0.0
for k in range(int(3.0 / m.opt.timestep)):
    mujoco.mj_step(m, d)
    if k % 250 == 0: emax = max(emax, abs(energy() - e0))
print(f"free double pendulum, 3 s: energy {e0:.5f} J, max drift {emax:.2e} J ({emax / abs(e0) * 100:.3f}%)")
assert emax / abs(e0) < 0.005
for b in (0.0044, 0.0175):
    m = mujoco.MjModel.from_xml_string(X.make(backlash=b, damping=0.0)); m.opt.gravity[:] = 0; d = mujoco.MjData(m)
    r = [m.joint(f"rotor{k}_j").qposadr[0] for k in (1, 2)]; l = [m.joint(f"j{k}").qposadr[0] for k in (1, 2)]
    rel = [[], []]
    for n in range(int(1.0 / m.opt.timestep)):
        t = n * m.opt.timestep; d.ctrl[:] = [0.05, 0.02] if t < 0.25 else [-0.05, -0.02] if t < 0.6 else [0, 0]
        mujoco.mj_step(m, d)
        for k in (0, 1): rel[k].append(d.qpos[r[k]] - d.qpos[l[k]])
    for k in (0, 1):
        w = max(rel[k]) - min(rel[k]); print(f"backlash {b:.4f}: joint {k + 1} relative angle width {w:.5f} ({w / b * 100:.1f}% of the gap)")
