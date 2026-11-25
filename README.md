# Four-Wheeled Holonomic Drive

Minimal STM32 sketch implementing inverse kinematics for a 4 omni-wheel holonomic base, built for a KRSBI Beroda soccer robot.

<!-- TODO: add demo GIF/photo -->

## Overview

A holonomic robot can translate in any direction and rotate at the same time, because its omni wheels roll freely sideways. This sketch is the first, standalone implementation of the 4-wheel kinematics I used for R2C's **KRSBI-B** (Kontes Robot Sepak Bola Indonesia Beroda) robot. It maps a body velocity `(vx, vy, vz)` to four signed motor commands and drives the H-bridges directly.

The same equations later went into the full robot firmware ([WheelBased-Soccer-Robot-Microcontroller](https://github.com/altilize/WheelBased-Soccer-Robot-Microcontroller)), which adds per-wheel PID, odometry and a PC link, and into [Manual-Holonomic-Control](https://github.com/altilize/Manual-Holonomic-Control) for gamepad teleoperation.

## Holonomic Kinematics

Wheels are mounted at 45°, 135°, 225° and 315° around the robot center. For wheel angle θ_i, the wheel speed is the body velocity projected onto the wheel's drive direction plus a rotation term:

```
v_i = -sin(θ_i) * vx + cos(θ_i) * vy + R * ω
```

In matrix form, with `sin/cos(45°) ≈ 0.707` and a 0.5 scale factor:

```
| A |   | -0.35   0.35   0.25 |   | vx |
| B | = | -0.35  -0.35   0.25 | * | vy |
| C |   |  0.35  -0.35   0.25 |   | vz |
| D |   |  0.35   0.35   0.25 |
```

Examples:

| Command | A | B | C | D | Result |
| --- | --- | --- | --- | --- | --- |
| `vy = 1` | + | - | - | + | Forward |
| `vx = 1` | - | - | + | + | Strafe sideways |
| `vz = 1` | + | + | + | + | Rotate in place |

Each signed result goes to `controlMotor()`: the sign picks which H-bridge input receives PWM, the magnitude (×100) sets the duty cycle.

## Features

- 4-wheel holonomic inverse kinematics in one function, `holonomic(vx, vy, vz)`.
- Bidirectional H-bridge control per wheel.
- Emergency switch on PA0 that disables the motor driver enable pin (PE7) and zeroes all PWM outputs.
- Serial printout of all four wheel commands for checking signs and wiring.

The loop drives a fixed forward command (`vx = 0, vy = 1, vz = 0`) as a wiring and direction test. Encoder pins and PID gains are declared for later closed-loop work but not used in this sketch.

## Hardware and Software Stack

| Item | Details |
| --- | --- |
| MCU | STM32F407 (STM32duino core) |
| Drive | 4 DC motors with omni wheels, H-bridge drivers |
| Safety | Emergency switch on PA0, shared motor enable on PE7 |
| Language | Arduino C++ |

## Repository Structure

```
four-wheeled-holonomic/
└── four-wheeled-holonomic.ino   # kinematics, motor control, emergency stop
```

## Build and Upload

1. Install the Arduino IDE and the [STM32duino core](https://github.com/stm32duino/Arduino_Core_STM32).
2. Open `four-wheeled-holonomic.ino`, select your STM32F407 board and upload.
3. Open the serial monitor at 9600 baud to see the wheel commands.
4. Change `x`, `y`, `z` in `loop()` to test other directions.

## My Role

I (Moses Jaguar Ersaptiarso) wrote this sketch at R2C as the starting point for the holonomic motion control of the KRSBI-B robot.
