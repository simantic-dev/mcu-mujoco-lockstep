# CAN master for the arm scenario. Protocol (see firmware/joint/src/main.cpp): it sends id 0x200 + joint, 4 bytes float32 little-endian,
# the target angle in rad from the rest pose; the joint answers with id 0x100 + joint status frames every 10 ms.
# Two steps. The peer only sends them; sim/plot.py judges the response from the logged hinge angle.
import math
import struct

from pmsm import MOTOR

JOINT = 1
STEPS = [(9_000_000, 0.3 * math.pi), (11_500_000, 0.6 * math.pi)]  # (virtual us, target rad from rest)
HOLD_US = 2_300_000


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.status = 0
        self.i = 0
        self.checking = False
        ctx.schedule_oneshot(STEPS[0][0])

    def on_timer(self):
        if not self.checking:
            _, rad = STEPS[self.i]
            self.ctx.send(0x200 + JOINT, struct.pack("<f", rad) + bytes(4))
            self.ctx.info(f"ARM_CMD target {rad:.4f} rad, hinge at {MOTOR.theta_m:.4f} rad")
            self.checking = True
            self.ctx.schedule_oneshot(HOLD_US)
            return
        self.ctx.info(f"ARM_END hold over, {self.status} status frames")
        self.checking = False
        self.i += 1
        if self.i < len(STEPS):
            self.ctx.schedule_oneshot(STEPS[self.i][0] - STEPS[self.i - 1][0] - HOLD_US)

    def on_frame(self, frame):
        if frame['id'] == 0x100 + JOINT:
            self.status += 1
