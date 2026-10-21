"""Motor, line-following, and goalkeeper control from the report."""

import math

from robot.parameters import (
    CORNER_BUMP_S,
    DEAD_PWM,
    DEPTH_MAX_CM,
    FRONT_OFFSET_CM,
    GOAL_KP,
    IMAGE_WIDTH,
    LINE_KD,
    LINE_KP,
    MAX_TPS_LOAD,
    PID_CUTOFF_HZ,
    PID_KD,
    PID_KI,
    PID_KP,
    PID_OUTPUT_LIMIT,
    REAR_OFFSET_CM,
    SAMPLE_S,
    SCALE_FACTOR,
    SHARP_TURN_ERROR_PX,
    URGENCY_MIN,
    WHEEL_RADIUS_CM,
)


def reference_tps(pwm):
    """Linear map printed in the report. Below the dead zone the wheel does not move."""
    if pwm <= DEAD_PWM:
        return 0.0
    return (pwm - DEAD_PWM) / SCALE_FACTOR


def ema_alpha(cutoff_hz, sample_s=SAMPLE_S):
    """Smoothing factor given in the report for cutoff fc and sample rate 1/dt."""
    fn = cutoff_hz * sample_s
    cosine = math.cos(2.0 * math.pi * fn)
    return cosine - 1.0 + math.sqrt(cosine * cosine - 4.0 * cosine + 3.0)


class SpeedPID:
    """Discrete PID with integral freeze at saturation and a filtered derivative."""

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD, cutoff_hz=PID_CUTOFF_HZ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = ema_alpha(cutoff_hz)
        self.integral = 0.0
        self.filtered = 0.0
        self.ready = False

    def update(self, measured_tps, reference_tps, dt=SAMPLE_S):
        error = reference_tps - measured_tps
        filtered = self.alpha * error + (1.0 - self.alpha) * self.filtered
        derivative = (filtered - self.filtered) / dt if self.ready else 0.0
        unsat = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-PID_OUTPUT_LIMIT, min(PID_OUTPUT_LIMIT, unsat))
        if output == unsat:
            self.integral += error * dt
        self.filtered = filtered
        self.ready = True
        return output


def wheel_references(speed_cm_s, yaw_rate, kappa=1.0):
    """Map robot velocity (v, omega) to three wheel rates, in the same unit as v/r."""
    radius = WHEEL_RADIUS_CM
    left = (speed_cm_s - REAR_OFFSET_CM * yaw_rate) / radius * kappa
    right = (speed_cm_s + REAR_OFFSET_CM * yaw_rate) / radius * kappa
    front = (FRONT_OFFSET_CM * yaw_rate) / radius * kappa
    return left, right, front


def line_yaw_rate(barycenter_x, previous_error=None, dt=SAMPLE_S):
    """PD heading correction. Error is the line's x position minus the image center."""
    error = barycenter_x - IMAGE_WIDTH / 2.0
    rate = 0.0 if previous_error is None else (error - previous_error) / dt
    return LINE_KP * error + LINE_KD * rate, error


def corner_override(error_px, wheel_tps):
    """Full-throttle bump when a sharp corner stalls every wheel."""
    stalled = all(abs(speed) <= 1e-9 for speed in wheel_tps)
    if abs(error_px) < SHARP_TURN_ERROR_PX or not stalled:
        return None
    direction = 1.0 if error_px > 0 else -1.0
    return {
        "duration_s": CORNER_BUMP_S,
        "wheel_tps": direction * MAX_TPS_LOAD,
    }


def urgency(depth_cm):
    """Closer balls demand a stronger move. Far balls stay at the floor of 0.3."""
    raw = (DEPTH_MAX_CM - depth_cm) / DEPTH_MAX_CM
    return max(URGENCY_MIN, min(1.0, raw))


def goalkeeper_speed(offset_cm, depth_cm):
    """Reactive lateral speed. Negative offset sends the robot the other way."""
    return -GOAL_KP * offset_cm * urgency(depth_cm)
