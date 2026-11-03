import math

from rc_vehicle_command.feasibility import make_feasible

L = 0.2
DELTA = math.radians(30)
MIN_V = 0.05


def feasible(v, omega):
    return make_feasible(v, omega, L, DELTA, MIN_V)


def test_feasible_command_unchanged():
    assert feasible(1.0, 0.5) == (1.0, 0.5)


def test_omega_clamped_to_max_curvature():
    max_omega = 1.0 / L * math.tan(DELTA)
    assert feasible(1.0, 100.0)[1] == max_omega
    assert feasible(1.0, -100.0)[1] == -max_omega


def test_reverse_keeps_omega_limit_symmetric():
    max_omega = 1.0 / L * math.tan(DELTA)
    assert feasible(-1.0, 100.0) == (-1.0, max_omega)


def test_in_place_rotation_dropped():
    assert feasible(0.0, 1.0) == (0.0, 0.0)
    assert feasible(0.01, 1.0) == (0.01, 0.0)
