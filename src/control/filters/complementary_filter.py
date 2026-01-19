# control/lib/filter.py
def complementary_filter_step(prev_angle, gyro, accel_angle, alpha, dt):
    return alpha*(prev_angle + gyro*dt) + (1-alpha)*accel_angle
