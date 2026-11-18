# CAN master for the two-link scenario: sends each joint its target angle (0x201 for joint 1, 0x202 for joint 2; float32 little-endian, rad from
# the hanging pose) at the times below, and counts the status frames (0x101, 0x102) the joints send back.
# Poses: arm out horizontal, then straight up (inverted), then the elbow kinked 0.35 rad while upright.
import math
import struct

LOG_EVERY = 25  # status frames per joint between log lines (the joints send one every 10 ms)

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
        self.ctx.info(f"ARM_CMD t={self.ctx.now_us / 1e6:.4f} shoulder {a:.4f} elbow {b:.4f} rad")
        self.i += 1
        if self.i < len(STEPS):
            self.ctx.schedule_oneshot(STEPS[self.i][0] - STEPS[self.i - 1][0])

    def on_frame(self, frame):
        if frame['id'] in (0x101, 0x102):
            j = frame['id'] - 0x101
            self.status[j] += 1
            if self.status[j] % LOG_EVERY == 0:  # the joint's own report: float32 angle from rest (rad), float32 speed (rad/s)
                angle, speed = struct.unpack('<ff', bytes(frame['data'][:8]))
                self.ctx.info(f"STATUS j{j + 1} t={self.ctx.now_us / 1e6:.4f} angle={angle:+.3f} speed={speed:+.2f}")
