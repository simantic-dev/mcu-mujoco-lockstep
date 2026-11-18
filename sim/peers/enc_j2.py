# The encoder of joint 2 of the two-link arm. It reads math.pi at the hanging pose (ENC_REST_ANGLE of that joint's firmware build).
import math

from mt6835_peer import Slave as _Slave


class Slave(_Slave):
    JOINT = 1
    OFFSET = math.pi
