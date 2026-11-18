# Rotor state shared by foc_plant.py (which integrates it) and mt6835_peer.py (which reads the
# shaft angle): a module of its own so both peer scripts import the same instance.
from collections import deque

P = 7


class Motor:
    def __init__(self):
        self.id = self.iq = 0.0
        self.wm = 0.0
        self.theta_m = 0.6 / P  # rotor starts 0.6 electrical rad off the d-axis alignment


MOTORS = [Motor(), Motor()]  # one per joint: the one-link scenario uses joint 0, the two-link scenario both
HISTS = [deque(maxlen=2048) for _ in MOTORS]  # sensed joint angle per plant tick, newest last; the encoder peer delays through it
READS_N = [[0] for _ in MOTORS]  # cumulative encoder angle reads by the firmware, per joint
MOTOR, HIST, READS = MOTORS[0], HISTS[0], READS_N[0]
