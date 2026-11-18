# The plant peer of one joint of the two-link arm: the same electrical model and 40 us exchange as arm_plant.py, but the mechanics are the
# shared MuJoCo world (world.py). Subclass it per joint (plant_j1.py, plant_j2.py) and set K = 0 or 1.
# MODELED SIMPLIFICATION: R, L, PSI, P are stand-ins (no particular motor); averaged switching; the bridge is always enabled.
import math
import os

import world
from pmsm import HISTS, MOTORS, READS_N

R, L, PSI, P = 0.2, 1e-4, 0.004, 7
VBUS = 12.0
TIM4 = 0x40000800
ARR, CCR1, CCR2, CCR4 = 0x2C, 0x34, 0x38, 0x40
STEP_US, SUB = 40, 10
SAMPLE_EVERY = 250  # 10 ms
ENC_SIDE = os.environ.get("ARM_ENC", "rotor")  # which hinge the encoders are mounted on: the motor's rotor or the load


class Plant:
    K = 0

    def __init__(self, ctx):
        self.ctx = ctx
        self.w = world.get()
        self.n = 0
        self.id = self.iq = 0.0
        k = self.K
        self.ie = self.w.load[k] if ENC_SIDE == "load" else self.w.rotor[k]
        MOTORS[k].theta_m, MOTORS[k].wm = float(self.w.d.qpos[self.ie]), 0.0
        ctx.schedule_periodic(STEP_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(STEP_US)
        req.value = 0

    def on_timer(self):
        k, w = self.K, self.w
        rd = lambda off: self.ctx.bus_read(TIM4 + off, 4)
        arr = rd(ARR) or 1
        duty = [min(rd(off) / arr, 1.0) for off in (CCR1, CCR2, CCR4)]
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        dt = STEP_US * 1e-6 / SUB
        wm, th_m, torque = float(w.d.qvel[w.rotor_v[k]]), float(w.d.qpos[w.rotor[k]]), 0.0
        for _ in range(SUB):
            th = P * th_m
            c = [math.cos(th - j * 2 * math.pi / 3) for j in range(3)]
            s = [math.sin(th - j * 2 * math.pi / 3) for j in range(3)]
            we = P * wm
            vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
            vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
            self.id += (vd - R * self.id + we * L * self.iq) / L * dt
            self.iq += (vq - R * self.iq - we * L * self.id - we * PSI) / L * dt
            torque += 1.5 * P * PSI * self.iq / SUB
            th_m += wm * dt
        w.submit(k, torque)
        # the hinges are periodic: after the firmware's power-up spin, report this joint within +-1.5 pi once it is nearly at rest (as arm_plant.py)
        ql = float(w.d.qpos[w.load[k]])
        if abs(ql) > 1.5 * math.pi and abs(float(w.d.qvel[w.load_v[k]])) < 0.3:
            turns = 2 * math.pi * round(ql / (2 * math.pi))
            w.d.qpos[w.load[k]] -= turns
            if w.rotor[k] != w.load[k]:
                w.d.qpos[w.rotor[k]] -= turns
        MOTORS[k].theta_m, MOTORS[k].wm = float(w.d.qpos[self.ie]), float(w.d.qvel[w.rotor_v[k]])
        HISTS[k].append(MOTORS[k].theta_m)
        self.n += 1
        if self.n % SAMPLE_EVERY == 0:
            self.ctx.info(f"J{k + 1} t={self.n * STEP_US / 1e6:.4f} q={w.d.qpos[w.load[k]]:.4f} qr={w.d.qpos[w.rotor[k]]:.4f} tau={torque:.4f} iq={self.iq:.3f} er={READS_N[k][0]}")
