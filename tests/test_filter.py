"""Unit tests for filter and controller primitives in eyantra-kb_3323."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.control.filters.complementary_filter import complementary_filter_step
from src.control.lqr.lqr import LQRController
from src.control.pid.pid import PIDController


def test_complementary_converge() -> None:
    angle = 0.0
    for _ in range(100):
        angle = complementary_filter_step(angle, gyro=0.01, accel_angle=0.009, alpha=0.98, dt=0.01)
    assert abs(angle - 0.01) < 0.02


def test_lqr_and_pid_controllers_behave_expectedly() -> None:
    lqr = LQRController(K=[10.0, 2.0])
    assert lqr.compute_control([0.1, 0.0]) == -1.0

    pid = PIDController(kp=10.0, ki=0.0, kd=1.0, deriv_alpha=0.5)
    out = pid.update(error=0.2, dt=0.01)
    assert out > 2.0
