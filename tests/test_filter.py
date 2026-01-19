# tests/test_filter.py
from control.lib.filter import complementary_filter_step
def test_complementary_converge():
    angle = 0.0
    for i in range(100):
        angle = complementary_filter_step(angle, gyro=0.01, accel_angle=0.009, alpha=0.98, dt=0.01)
    assert abs(angle - 0.01) < 0.02
