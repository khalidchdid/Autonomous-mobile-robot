"""Numbers taken from the May 2025 Modal report and the Casillas defense."""

# Geometry
ROBOT_WIDTH_CM = 20.0
ROBOT_LENGTH_CM = 30.0
ROBOT_HEIGHT_CM = 8.0
REAR_OFFSET_CM = 11.5
FRONT_OFFSET_CM = 13.0
WHEEL_RADIUS_CM = 2.8
CAMERA_HEIGHT_CM = 8.0

# DG01D-E motors and Grove I2C L298P drivers
TICKS_PER_REV = 144
DEAD_PWM = 40
PWM_MAX = 255
SCALE_FACTOR = (PWM_MAX - DEAD_PWM) / PWM_MAX
MAX_TPS_NO_LOAD = 120.0
MAX_TPS_LOAD = 90.0
SPEED_RESOLUTION_TPS = 10.0
SAMPLE_S = 0.1

I2C_REAR = 0x0F
I2C_FRONT = 0x0D
REG_PWM = 0x82
REG_DIRECTION = 0xAA
REAR_FORWARD = 0x06
REAR_BACKWARD = 0x09
REAR_LEFT = 0x0A
REAR_RIGHT = 0x05
FRONT_LEFT = 0x06
FRONT_RIGHT = 0x09

# Discrete-time speed PID, with an EMA on the derivative
PID_KP = 0.3
PID_KI = 0.005
PID_KD = 0.005
PID_CUTOFF_HZ = 4.0
PID_OUTPUT_LIMIT = 255.0

# Line follower. The defense slides list kd = 0.001; the report lists 0.01.
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FRAME_RATE = 30
LINE_KP = 0.005
LINE_KD = 0.01
SHARP_TURN_ERROR_PX = 172
CORNER_BUMP_S = 0.1

# OpenCV HSV ranges. Red needs both ends of the hue circle.
HSV_RED = (
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (180, 255, 255)),
)
HSV_BLUE = ((100, 150, 50), (140, 255, 255))

# Goalkeeper vision and control
BALL_DIAMETER_CM = 4.6
PINHOLE_K = 346.1
PINHOLE_K_TIMES_D = 1592.06
DEPTH_MAX_CM = 100.0
URGENCY_MIN = 0.3
GOAL_KP = 100.0
LATERAL_RANGE_CM = 50.0
SHOT_DISTANCE_CM = (50.0, 70.0)

# Rule used to predict a successful stop
RULE_ANGLE_DEG = 20.0
RULE_SPEED_CM_S = 50.0
RULE_DISTANCE_CM = 60.0
STOP_ERROR_CM = 10.0

# Printed regression: E = a0 + a1 d + a2 theta + a3 v
REGRESSION = (-15.82, 0.082, 0.205, 0.227)
