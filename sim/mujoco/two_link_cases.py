# Cases for the two-link arm (run_two_link.py). `src` as in cases.py: datasheet / catalogue / chosen.
import math
DEG = math.pi / 180
AS = dict(ENC_BITS=14, ENC_NOISE_RAD=0.068 * DEG, ENC_DELAY_US=110)  # AS5047P datasheet values, as in cases.py
CASES = [
    dict(name="ideal", label="Ideal", params="exact encoders, no friction, no backlash", arm={}, env={}),
    dict(name="as5047p", label="AS5047P-class encoders", params="14-bit, noise 0.068 deg rms, delay 110 us, both joints", arm={}, env=dict(AS), src="datasheet (AS5047P)"),
    dict(name="rough_sensor", label="Rough encoders", params="12-bit, noise 0.3 deg rms, delay 5 ms, both joints", arm={}, env=dict(ENC_BITS=12, ENC_NOISE_RAD=0.3 * DEG, ENC_DELAY_US=5000), src="chosen"),
    dict(name="lag_30ms", label="Encoder lag", params="delay 30 ms on both encoders, otherwise exact", arm={}, env=dict(ENC_DELAY_US=30000), src="chosen"),
    dict(name="lash_motor", label="Backlash 1 deg, encoders on motors", params="gap 1 deg at both joints, encoders on the motors", arm=dict(backlash=1 * DEG), env={}, src="chosen"),
    dict(name="lash_load", label="Backlash 1 deg, encoders on loads", params="gap 1 deg at both joints, encoders on the loads (gap inside the loop)", arm=dict(backlash=1 * DEG), env=dict(ARM_ENC="load"), src="chosen"),
    dict(name="heavy_tip", label="Heavier tip", params="tip mass 0.06 kg (shoulder gravity torque up to 0.47 N*m: over the GL40's 0.25 rated, under its 0.75 peak)", arm=dict(tip_mass=0.06), env={}, src="chosen"),
    dict(name="combined", label="Combined", params="AS5047P-class encoders + 1 deg gap (encoders on motors) + friction 0.02 N*m", arm=dict(backlash=1 * DEG, frictionloss=0.02), env=dict(AS), src="mix of the above"),
]
