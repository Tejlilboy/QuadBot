# QuadBot 🕷️
![QuadBot](images/quadbot.png)


### ESP32-Based Wireless Quadruped Robot

QuadBot is a custom-built, four-legged robotic platform powered by an ESP32 and eight independently controlled servo motors.

The robot features wireless Xbox controller support, multiple movement patterns, and an animated OLED face that responds to its movements and current state.

The project combines embedded systems programming, electronics design, PCB development, servo control, and mechanical robotics into a single interactive platform.

---

## Features

- **8-Servo Locomotion:** Independent control of four legs using eight hobby servos.
- **Wireless Controller Support:** Bluetooth connectivity through Bluepad32.
- **Multiple Movement Modes:** Forward, backward, turning, and lateral crab walking.
- **Smooth Servo Motion:** Rate-limited servo positioning for smoother transitions.
- **Custom Power Distribution PCB:** Dedicated PCB designed for the robot's power distribution.
- **Animated OLED Face:** Custom facial expressions and animations using an SSD1306 display.
- **Interactive Controls:** Facial expressions and movements respond to controller inputs.
- **Autonomous Idle Behavior:** Random idle animations and automatic sleep after inactivity.
- **Additional Animations:** Dancing, waving, push-ups, bowing, shaking, and more.

---

## Hardware

| Component | Description |
|---|---|
| Microcontroller | Seeed XIAO ESP32-S3 |
| Servo Motors | 8x MG90S Servos |
| Display | SSD1306 128x64 OLED |
| Controller | Xbox Wireless Controller |
| Communication | Bluetooth |
| Display Interface | I2C |
| Power Distribution | Custom-designed PCB |

## Custom Power Distribution PCB
![Schematic Capture](images/quadbot-schematic.png)

![PCB layout](images/quadbot-pcb.png)

![Unsoldered PCB](images/PCB-unsoldered.png)

![Soldered PCB](images/PCB-soldered.png)






### Servo Configuration

Each leg uses two servos:

- **Hip Servo:** Controls forward and backward leg movement.
- **Lift Servo:** Raises and lowers the leg.

The eight servos are independently controlled using the ESP32's LEDC PWM hardware.

| Servo | Leg | Function | GPIO | Home | Extended |
|---|---|---|---|---:|---:|
| R1 | Front Right | Hip | D0 | 0° | 90° |
| R2 | Rear Right | Hip | D2 | 90° | 0° |
| L1 | Front Left | Hip | D7 | 90° | 0° |
| L2 | Rear Left | Hip | D9 | 0° | 90° |
| R3 | Front Right | Lift | D1 | 180° | 0° |
| R4 | Rear Right | Lift | D3 | 0° | 180° |
| L3 | Front Left | Lift | D8 | 0° | 180° |
| L4 | Rear Left | Lift | D10 | 180° | 0° |

*Servo calibration values are specific to the current mechanical configuration.*

### OLED Connections

| OLED Pin | ESP32 Pin |
|---|---|
| SDA | D4 |
| SCL | D5 |
| Address | 0x3C |

---

## Power Distribution

QuadBot uses a custom-designed power distribution PCB to organize and distribute power throughout the robot.

The PCB was designed specifically for the project's electrical requirements, providing a centralized power distribution solution for the microcontroller and servo system.

This was an important part of the hardware development, particularly given the current requirements of operating eight servo motors and the size constraints of the body.

---

## Controller Mapping

QuadBot uses an Xbox wireless controller connected through Bluetooth.

| Input | Function |
|---|---|
| Left Stick Up | Walk Forward |
| Left Stick Down | Walk Backward |
| Left Stick Left | Turn Left |
| Left Stick Right | Turn Right |
| Right Stick | Control OLED Eyes |
| Right Trigger (RT) | Sprint |
| Left Trigger (LT) | Sneak |
| Left Bumper (LB) | Crab Walk Left |
| Right Bumper (RB) | Crab Walk Right |
| A | Sit / Stand |
| B (Hold) | Dance |
| X | Play Dead |
| Y (Hold) | Wave |
| D-Pad Up (Hold) | Push-Ups |
| D-Pad Down | Sleep / Wake |
| D-Pad Left (Hold) | Shake |
| D-Pad Right (Hold) | Bow |

The robot also features automatic idle behavior. After 60 seconds without controller input, it performs a yawn animation before entering sleep mode.

---

## Software Architecture

The firmware is organized into several functional sections.

| Module | Description |
|---|---|
| Servo Driver | Custom servo control using ESP32 LEDC PWM |
| Motion Control | Movement sequences and servo positioning |
| Controller Interface | Bluetooth controller input through Bluepad32 |
| OLED Engine | Facial expressions and animations |
| Timing System | Movement timing and speed adjustments |
| State Management | Sitting, sleeping, and play-dead states |

### Servo Control

QuadBot uses a custom servo driver built around the ESP32 LEDC peripheral.

Each servo receives an independent PWM channel operating at 50 Hz with 14-bit resolution.

The driver implements:

- Independent servo positioning.
- Target-angle control.
- Rate-limited movement.
- Configurable movement speeds.
- PWM pulse-width conversion.

This allows the robot to transition between positions without immediately jumping to the target angle.

### Movement System

Movement is implemented using predefined servo-position sequences, allowing multiple joints to coordinate during locomotion and animations.

The firmware supports adjustable gait timing, movement speeds, and directional correction to compensate for mechanical differences between the robot's left and right sides.

### OLED Animation Engine

The OLED face is generated programmatically using the Adafruit GFX drawing library.

Rather than displaying static images, the firmware draws facial features using geometric primitives.

The display includes:

- Idle expressions.
- Blinking and eye movement.
- Happy and surprised expressions.
- Sleep animations.
- Movement-specific expressions.
- Controller connection indicators.
- Trigger-dependent visual effects.

The right analog stick also allows direct control of the character's eyes.

---

## Software Requirements

### Development Environment
- Arduino IDE
- ESP32 / Bluepad32 board package

### Required Libraries

- [Bluepad32](https://github.com/ricardoquesada/bluepad32)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

---

## Project Development

This project involved the design and integration of several hardware and software components.

Key areas of development included:

- ESP32 peripheral configuration.
- Custom power distribution PCB design.
- Component selection and hardware integration.
- PCB layout and electrical connections.
- PWM signal generation.
- Bluetooth communication.
- Embedded C++ programming.
- Real-time controller input processing.
- Servo calibration and synchronization.
- I2C communication.
- OLED graphics programming.
- Coordinated robotic movement.

---

## Future Improvements

Potential future developments include:

- Improved walking stability and gait optimization.
- Inverse kinematics for more precise foot positioning.
- Obstacle detection and environmental sensing.
- Autonomous movement.
- Additional movement sequences.
- Improved mechanical design.

---

## Project Status

**Status: Functional Prototype**

QuadBot currently supports wireless manual control, multiple movement sequences, interactive OLED animations, and autonomous idle behavior.

Further improvements will focus on locomotion, stability, and additional autonomous capabilities.

---

## Credits

The original QuadBot concept and 3D-printable STL files were created by [**Dorian Todd and his Sesame Project**](https://github.com/dorianborian/sesame-robot). The electronics, component selection, custom power distribution PCB, firmware, software, and overall hardware implementation are my own work.

---

*Designed, assembled, programmed, and developed as a personal robotics project.*