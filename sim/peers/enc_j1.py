# The encoder of joint 1 of the two-link arm. It reads math.pi / 2 at the hanging pose (ENC_REST_ANGLE of that joint's firmware build).
import math

from mt6835_peer import Slave as _Slave


class Slave(_Slave):
    JOINT = 0
    OFFSET = math.pi / 2
