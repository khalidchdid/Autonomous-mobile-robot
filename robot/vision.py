"""Geometry used to turn one camera frame into a ball position."""

from robot.parameters import BALL_DIAMETER_CM, PINHOLE_K


def depth_cm(diameter_px):
    """Pinhole depth z = K * D / d, with D the real ball diameter."""
    if diameter_px <= 0:
        raise ValueError("apparent diameter must be positive")
    return PINHOLE_K * BALL_DIAMETER_CM / diameter_px


def lateral_offset_cm(center_x, image_center_x, diameter_px):
    """Real lateral offset from the pixel offset and the ball's apparent size."""
    if diameter_px <= 0:
        raise ValueError("apparent diameter must be positive")
    return (center_x - image_center_x) / diameter_px * BALL_DIAMETER_CM
