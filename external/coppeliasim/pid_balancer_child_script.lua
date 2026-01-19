
#-- TASK 1B code for coppeliasim child script --
# PID balancer for a two-wheeled robot

import math

def sysCall_init():
    sim = require('sim')

    # --- Get handles ---
    self.body = sim.getObject('/body')
    self.left_joint = sim.getObject('/body/left_joint')
    self.right_joint = sim.getObject('/body/right_joint')

    # --- PID gains (tune these) ---
    self.Kp = 60.0
    self.Ki = 0.02   # set to 0.0 while you tune PD
    self.Kd = 2.0
    self.Imax = 3.0  # max magnitude of the integral term (i_term)

    # --- Output limits ---
    self.max_target_vel = 8.0   # clamp PID output (rad/s)

    # --- Derivative filter (low-pass) ---
    self.derivative_alpha = 0.7
    self.prev_d_filtered = 0.0

    # --- Helper variables ---
    self.ref = 0.0         # reference upright (roll = 0)
    self.integral = 0.0    # integrates raw error (not multiplied by Ki)
    self.prev_err = 0.0
    self.prev_time = sim.getSimulationTime()

    # Optional logging throttle
    self._log_counter = 0

    print("PID Balancer Initialized: Kp={}, Ki={}, Kd={}".format(self.Kp, self.Ki, self.Kd))


def sysCall_actuation():
    sim = require('sim')

    # --- Timing ---
    t = sim.getSimulationTime()
    dt = t - self.prev_time
    if dt <= 0:
        return

    # --- Measure orientation (roll from body) ---
    euler = sim.getObjectOrientation(self.body, -1)
    roll = euler[0]  # radians

    # --- Error ---
    err = self.ref - roll

    # --- Proportional ---
    p_term = self.Kp * err

    # --- Derivative (filtered) ---
    raw_d = (err - self.prev_err) / dt
    d_filtered = self.derivative_alpha * self.prev_d_filtered + (1.0 - self.derivative_alpha) * raw_d
    d_term = self.Kd * d_filtered

    # --- Integral (anti-windup) ---
    # integrate raw error and compute i_term = Ki * integral
    # clamp i_term to +/- Imax
    if self.Ki != 0.0:
        self.integral += err * dt
        i_term = self.Ki * self.integral
        # clamp i_term
        if i_term > self.Imax:
            i_term = self.Imax
            # prevent further wind-up past clamp by capping integral
            self.integral = self.Imax / self.Ki
        elif i_term < -self.Imax:
            i_term = -self.Imax
            self.integral = -self.Imax / self.Ki
    else:
        i_term = 0.0

    # --- PID output before saturation ---
    pid_raw = p_term + i_term + d_term

    # --- Saturate output to achievable joint velocity ---
    if pid_raw > self.max_target_vel:
        pid = self.max_target_vel
    elif pid_raw < -self.max_target_vel:
        pid = -self.max_target_vel
    else:
        pid = pid_raw

    # If your wheel/actuator mapping requires inversion uncomment next line:
    # pid = -pid

    # --- Apply to motors (symmetrically) ---
    sim.setJointTargetVelocity(self.left_joint, pid)
    sim.setJointTargetVelocity(self.right_joint, pid)

    # --- Update memory ---
    self.prev_err = err
    self.prev_time = t
    self.prev_d_filtered = d_filtered

    # --- Optional light logging (reduces console spam) ---
    self._log_counter += 1
    if self._log_counter >= 20:  # print every 20 actuation steps
        # print roll in degrees and PID terms for quick debugging
        print("roll(deg)={:.2f}, P={:.3f}, I={:.3f}, D={:.3f}, out={:.3f}".format(
            math.degrees(roll), p_term, i_term, d_term, pid))
        self._log_counter = 0


def sysCall_sensing():
    pass


def sysCall_cleanup():
    sim = require('sim')
    sim.setJointTargetVelocity(self.left_joint, 0)
    sim.setJointTargetVelocity(self.right_joint, 0)
    print("Motors stopped.")