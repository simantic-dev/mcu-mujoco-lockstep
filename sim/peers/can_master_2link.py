# CAN master for the two-link scenario: sends each joint its target angle (0x201 for joint 1, 0x202 for joint 2; float32 little-endian, rad from
# the hanging pose) at the times below, and counts the status frames (0x101, 0x102) the joints send back.
# Poses: arm out horizontal, then straight up (inverted), then the elbow kinked 0.35 rad while upright.
import math
import struct

STEPS = [(9_500_000, (0.5 * math.pi, 0.0)), (12_000_000, (math.pi, 0.0)), (14_500_000, (math.pi, 0.35))]  # (virtual us, (shoulder, elbow) rad)


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.status = [0, 0]
        self.i = 0
        ctx.schedule_oneshot(STEPS[0][0])

    def on_timer(self):
        _, (a, b) = STEPS[self.i]
        self.ctx.send(0x201, struct.pack("<f", a) + bytes(4))
        self.ctx.send(0x202, struct.pack("<f", b) + bytes(4))
        self.ctx.info(f"ARM_CMD shoulder {a:.4f} elbow {b:.4f} rad")
        self.i += 1
        if self.i < len(STEPS):
            self.ctx.schedule_oneshot(STEPS[self.i][0] - STEPS[self.i - 1][0])

    def on_frame(self, frame):
        if frame['id'] in (0x101, 0x102):
            self.status[frame['id'] - 0x101] += 1
