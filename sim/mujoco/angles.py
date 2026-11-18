# Display helper: a joint angle that spins many turns, drawn as an angle within +-pi, with a gap (NaN) wherever it jumps across the wrap.
import numpy as np


def wrapped(q, center=0.0):
    # within center +- pi
    q = np.asarray(q, float)
    w = (q - center + np.pi) % (2 * np.pi) - np.pi + center
    out = w.copy()
    out[1:][np.abs(np.diff(w)) > np.pi] = np.nan
    return out
