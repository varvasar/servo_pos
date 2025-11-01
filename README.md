# 🎯 servo_pos — Continuous Rotation Servo Positioning Library for Raspberry Pi 5

**Version:** 0.01  
**License:** GPL v3  

`servo_pos` is a Python library that allows you to **control continuous rotation servo motors in a positioning style** — just like standard positional servos (not very precise of course).  
It uses the [`lgpio`](https://abyz.me.uk/lg/py_lgpio.html) library on the **Raspberry Pi 5** to generate PWM signals and manage multiple servos simultaneously.

---

## 🧭 What It Does

Unlike ordinary continuous rotation servo control (which only sets direction and speed),  
**`servo_pos` lets you rotate a continuous servo by a defined angle** — thanks to an **interpolated rotation-time model**.

This makes it possible to use affordable continuous servos in projects where approximate angular positioning is sufficient.

---

For calibration, the module uses the `DEFAULT_ROTATION_TIME` dictionary, where the keys represent rotation speeds and the values represent the time in seconds for a full 360° rotation; you can update this dictionary to calibrate your own motor, but it must include at least two pairs: the minimum speed and the maximum speed.

---
## 🚀 Features

- 🌀 Control **continuous rotation servos by angle or speed**
- ⚙️ Built-in **angle-to-time interpolation** for consistent motion
- 🔁 Manage **multiple servos simultaneously**
- 🧵 Thread-safe operation with per-pin locks
- 🧱 Simple `ServoMotor` class for single control
- 🧩 `ServoController` for managing multiple motors
- 💡 Easy cleanup and safe GPIO resource management
- ✅ Context manager support (`with` syntax)

---

## Basic examples

```python
from crservo_pos import ServoMotor
import time

servo = ServoMotor(pin=18)
servo.rotate(45)   # Rotate about +45°
servo.rotate(-90)  # Rotate about -90°
servo.cleanup()
```


```python
from crservo_pos import ServoMotor

with ServoMotor(pin=18) as servo:
    servo.rotate(90)
```

```python
from crservo_pos import ServoController

with ServoController() as controller:
    controller.add_servo("left", pin=17)
    controller.add_servo("right", pin=18)

    # Rotate both simultaneously
    controller.rotate_all(45)

    # Access and control one servo
    left = controller.get_servo("left")
    if left:
        left.rotate(-30)
```

