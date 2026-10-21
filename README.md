# Autonomous mobile robot

A Raspberry Pi robot first follows a red and blue line, then plays goalkeeper as Casillas: one fixed camera, three DC motors, and a reactive intercept of a red ball.

The lab folder contained the written report, the defense slides, a Grove I2C motor-driver library, a short C++ exercise, and two OpenCV links. It did not contain the onboard robot program. This repo keeps the project’s measurements, control laws, and the 18 recorded shots in code.

## Platform

The chassis is 20 cm by 30 cm by 8 cm. A Raspberry Pi 4 and a PiCamera V2 (640×480 at 30 fps) sit 8 cm off the ground. Two rear DG01D-E motors and one front motor are driven by Grove L298P boards over I2C, using wiringPi. Rear wheels are at address `0x0f`, the front wheel at `0x0d`. Speed is register `0x82`. Direction is register `0xaa`.

Hall sensors on the motors count 144 ticks per revolution. Speed is ticks per second over a 0.1 s window, so the loop does not pay for a conversion to revolutions. Unloaded tops out at 120 TPS, loaded at 90 TPS, and the reading jitters by about 10 TPS. PWM below 40 does nothing. The report’s linear correction above that dead zone is

```text
scale = (255 - 40) / 255
tps = (pwm - 40) / scale
```

Direction bytes are `0x06` forward, `0x09` backward, `0x0a` left, and `0x05` right on the rear driver. The front driver uses `0x06` for left and `0x09` for right. A speed PID (`kp = 0.3`, `ki = 0.005`, `kd = 0.005`, derivative cutoff 4 Hz) closes the loop. The integral is frozen while the output is saturated.

Image work uses OpenCV, with [thresholding](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html) and [image moments](https://docs.opencv.org/3.4/d0/d49/tutorial_moments.html). Frames come from the Pi camera through LCCV. A frame takes about 20–30 ms.

## Line follower

Red uses two HSV ranges, `(0, 120, 70)–(10, 255, 255)` and `(170, 120, 70)–(180, 255, 255)`. Blue uses `(100, 150, 50)–(140, 255, 255)`. The line’s barycenter gives the heading error `e = x - 320`.

Wheel rates come from the robot velocity `(v, ω)`, wheel radius 2.8 cm, rear offset 11.5 cm, and front offset 13.0 cm:

```text
ω1 = (v - 11.5 ω) / r · κ
ω2 = (v + 11.5 ω) / r · κ
ω3 = (13.0 ω) / r · κ
```

Heading is a PD law, `ω = 0.005 e + 0.01 ė`. The defense slides list the derivative gain as 0.001. When the error reaches about 172 px and all three wheels stall, the robot drops PID for 0.1 s and drives at the loaded maximum, 90 TPS, toward the turn. Vision, motor PID, and that corner override run on separate threads, with one flag so the override is not fought by the PID.

## Goalkeeper

The robot can only slide sideways, over about 50 cm. Shots are taken from 50–70 cm with a ball 4.6 cm across. The largest red contour is kept, opened morphologically, and fitted with a circle of pixel diameter `d`.

```text
z = 346.1 · 4.6 / d
x = (u - cx) / d · 4.6
```

`z` is depth in centimeters and `x` is the lateral miss. Predicting the crossing point from two frames, or from a fitted line, was too noisy and too late, so the robot only reacts to the current position:

```text
urgency = clamp((100 - z) / 100, 0.3, 1)
v = -100 · x · urgency
```

Each frame captures, measures `(x, z)`, applies that law, compensates the PWM dead zone, and sends direction and speed to the driver.

## Did it stop the ball?

A stop means the ball arrived within 10 cm of the robot’s center. The report’s rule predicts a stop when the shot is within 20° of straight, slower than 50 cm/s, and taken from at least 60 cm. On the 18 trials in `data/trials.csv` that rule scores 55.6%, with 0 false alarms and 8 misses. Short, fast shots are the ones that get through.

The printed error fit is `E = -15.82 + 0.082 d + 0.205 θ + 0.227 v`, with a stated MAE of about 4.4 cm. Speed and angle dominate the miss. The conclusion is that the motors cannot reach the roughly 2 m/s a fast shot from 70 cm would demand. A later version would need a faster drive, a real-time microcontroller, and a linear rail instead of wheels.

## Run

No packages beyond Python 3:

```bash
python -m robot.evaluate
```

`robot/control.py` and `robot/vision.py` are the same laws as the report, as plain functions. `robot/parameters.py` holds the calibrated constants.
