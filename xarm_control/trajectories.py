"""Time scaling; no robot dependencies."""
import numpy as np


def quintic(start, goal, t, duration):
    """Return joint position, velocity, acceleration; zero derivatives at endpoints."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if duration <= 0:
        raise ValueError('duration must be positive')
    u = np.clip(float(t) / duration, 0.0, 1.0)
    s = 10*u**3 - 15*u**4 + 6*u**5
    sd = (30*u**2 - 60*u**3 + 30*u**4) / duration
    sdd = (60*u - 180*u**2 + 120*u**3) / duration**2
    delta = goal - start
    return start + delta*s, delta*sd, delta*sdd


def cubic(start, goal, t, duration):
    """Zero endpoint speeds; nonzero endpoint acceleration."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if duration <= 0:
        raise ValueError('duration must be positive')
    u = np.clip(float(t)/duration, 0.0, 1.0)
    s = 3*u*u - 2*u**3
    sd = (6*u - 6*u*u)/duration
    sdd = (6 - 12*u)/duration**2
    return start+(goal-start)*s, (goal-start)*sd, (goal-start)*sdd
