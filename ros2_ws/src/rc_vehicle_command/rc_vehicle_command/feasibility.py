import math


def make_feasible(v, omega, wheelbase, max_steering_angle, min_linear_speed):
    """
    Return a (v, omega) pair satisfying the bicycle-model constraint.

    |omega| <= |v| / wheelbase * tan(max_steering_angle)

    Below min_linear_speed the car cannot turn, so omega is dropped
    (in-place rotation requests are not executed).
    """
    if abs(v) < min_linear_speed:
        return v, 0.0
    max_omega = abs(v) / wheelbase * math.tan(max_steering_angle)
    return v, max(-max_omega, min(max_omega, omega))
