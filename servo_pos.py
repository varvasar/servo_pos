"""
Servo motor control module for Raspberry Pi 5 using lgpio.

This module provides a ServoMotor class for controlling continuous rotation
servo motors with support for multiple simultaneous instances.
"""

import lgpio
import logging
import threading
import time

from typing import Dict, Tuple, Optional

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ServoMotorError(Exception):
    """Base exception for servo motor errors."""
    pass


class ServoMotor:
    """
    A class to control continuous rotation servo motors on Raspberry Pi 5.

    Attributes:
        DEFAULT_PWM_FREQUENCY (int): Default PWM frequency in Hz
        DEFAULT_STOP_DUTY (int): Default stop pulse width in microseconds
        DEFAULT_MIN_ANGLE (int): Default minimum rotation angle in degrees
    """

    DEFAULT_PWM_FREQUENCY = 50
    DEFAULT_STOP_DUTY = 1500
    DEFAULT_MIN_ANGLE = 10

    # Default rotation time mapping for different speeds
    DEFAULT_ROTATION_TIME = {
        0.1: 2.4,
        0.2: 2.0,
        0.3: 1.85,
        0.5: 1.10,
        1.0: 0.71,
        1.1: 0.60,
        1.3: 0.56,
        1.4: 0.46,
        1.5: 0.44
    }

    # Class-level tracking for pin usage only
    _pin_usage = {}  # Track which pins are in use
    _pin_locks = {}  # Individual locks for each pin

    def __init__(
            self,
            pin: int,
            pwm_frequency: int = DEFAULT_PWM_FREQUENCY,
            stop_duty: int = DEFAULT_STOP_DUTY,
            min_angle: int = DEFAULT_MIN_ANGLE,
            rotation_time_map: Optional[Dict[float, float]] = None,
            chip_num: int = 0
    ):
        """
        Initialize a ServoMotor instance.

        Args:
            pin: GPIO pin number for the servo
            pwm_frequency: PWM frequency in Hz (default: 50)
            stop_duty: Stop pulse width in microseconds (default: 1500)
            min_angle: Minimum rotation angle in degrees (default: 10)
            rotation_time_map: Custom speed to rotation time mapping
            chip_num: GPIO chip number (default: 0)

        Raises:
            ServoMotorError: If the pin is already in use or GPIO initialization fails
        """
        self.pin = pin
        self.pwm_frequency = pwm_frequency
        self.stop_duty = stop_duty
        self.min_angle = min_angle
        self.chip_num = chip_num
        self.rotation_time_map = rotation_time_map or self.DEFAULT_ROTATION_TIME.copy()
        self._is_initialized = False

        # Check if pin is already in use
        if pin in self._pin_usage and self._pin_usage[pin]:
            raise ServoMotorError(f"Pin {pin} is already in use by another servo instance")

        # Create a REENTRANT lock for this pin if it doesn't exist
        if pin not in self._pin_locks:
            self._pin_locks[pin] = threading.RLock()

        self._pin_lock = self._pin_locks[pin]

        # Initialize the servo
        self._initialize()

    def _initialize(self) -> None:
        """Initialize the servo and mark the pin as in use."""
        try:
            self._pin_usage[self.pin] = True
            self._is_initialized = True
            logger.info(f"Initialized servo on pin {self.pin}")

            # Set to stop position
            self._set_pwm_stop()

        except Exception as e:
            raise ServoMotorError(f"Failed to initialize servo on pin {self.pin}: {e}")

    def _set_pwm(self, pulse_width: int, duration: Optional[float] = None) -> None:
        """
        Set PWM signal for the servo with open/close for each operation to avoid jiggling.

        Args:
            pulse_width: Pulse width in microseconds
            duration: Optional duration to run the PWM before stopping (in seconds)
        """
        if not self._is_initialized:
            raise ServoMotorError("Servo not initialized")

        h = None
        try:
            # Open chip and claim output
            h = lgpio.gpiochip_open(self.chip_num)
            lgpio.gpio_claim_output(h, self.pin)

            duty_cycle = (pulse_width / 20000) * 100  # Convert to duty cycle percentage
            lgpio.tx_pwm(h, self.pin, self.pwm_frequency, duty_cycle)

            # If duration is specified, keep PWM running for that time
            if duration is not None and duration > 0:
                time.sleep(duration)
                # Stop PWM before closing
                lgpio.tx_pwm(h, self.pin, self.pwm_frequency, self.stop_duty / 20000 * 100)

            # Close the handle
            lgpio.gpiochip_close(h)

        except Exception as e:
            if h is not None:
                try:
                    lgpio.gpiochip_close(h)
                except:
                    pass
            logger.error(f"Failed to set PWM on pin {self.pin}: {e}")
            raise ServoMotorError(f"Failed to set PWM: {e}")

    def _set_pwm_stop(self) -> None:
        """Internal method to stop the servo."""
        self._set_pwm(self.stop_duty)

    def rotate_with_speed(self, angle: float, speed: float) -> None:
        """
        Rotate the servo by a specific angle at a specific speed.

        Args:
            angle: Rotation angle in degrees (positive for forward, negative for reverse)
            speed: Speed value from 0.0 to 1.5 (mapped to pulse width range)

        Raises:
            ValueError: If speed is not in valid range
        """
        if angle == 0:
            return

        if not 0.0 <= speed <= 1.5:
            raise ValueError("Speed must be between 0.0 and 1.5")

        with self._pin_lock:
            # Find the rotation time for this speed from the mapping
            speeds = sorted(self.rotation_time_map.keys())

            if speed in self.rotation_time_map:
                time_360 = self.rotation_time_map[speed]
            elif speed <= speeds[0]:
                time_360 = self.rotation_time_map[speeds[0]]
            elif speed >= speeds[-1]:
                time_360 = self.rotation_time_map[speeds[-1]]
            else:
                # Interpolate between two closest speeds
                for i in range(len(speeds) - 1):
                    s1, s2 = speeds[i], speeds[i + 1]
                    if s1 <= speed <= s2:
                        t1, t2 = self.rotation_time_map[s1], self.rotation_time_map[s2]
                        time_360 = t1 + (t2 - t1) * ((speed - s1) / (s2 - s1))
                        break

            # Calculate time needed for the requested angle
            time_to_angle = abs(angle) * time_360 / 360

            # Determine direction and pulse width
            direction = 1 if angle > 0 else -1
            pulse_width = self.stop_duty + direction * int(speed * 500)

            # Execute rotation with duration
            logger.debug(f"Rotating servo {self.pin} by {angle}° at speed {speed} for {time_to_angle}s")
            self._set_pwm(pulse_width, duration=time_to_angle)
            logger.debug(f"Stopped servo {self.pin} after rotation")

    def _calculate_speed_and_time(self, angle: float) -> Tuple[float, float]:
        """
        Calculate speed and rotation time for a given angle.

        Args:
            angle: Rotation angle in degrees

        Returns:
            Tuple of (speed, time_for_360_degrees)
        """
        # Clamp angle to valid range
        angle = max(self.min_angle, min(angle, 360))

        # Extract and sort speeds
        speeds = sorted(self.rotation_time_map.keys())
        min_speed, max_speed = speeds[0], speeds[-1]

        # Linear mapping: min_angle° → min_speed, angle ≥90° → max_speed
        if angle >= 90:
            speed = max_speed
        else:
            speed = min_speed + (max_speed - min_speed) * ((angle - self.min_angle) / (90 - self.min_angle))

        # Interpolate rotation time
        if speed <= min_speed:
            time_360 = self.rotation_time_map[min_speed]
        elif speed >= max_speed:
            time_360 = self.rotation_time_map[max_speed]
        else:
            # Find two closest speeds for interpolation
            for i in range(len(speeds) - 1):
                s1, s2 = speeds[i], speeds[i + 1]
                if s1 <= speed <= s2:
                    t1, t2 = self.rotation_time_map[s1], self.rotation_time_map[s2]
                    # Linear interpolation
                    time_360 = t1 + (t2 - t1) * ((speed - s1) / (s2 - s1))
                    break

        return round(speed, 3), round(time_360, 3)

    def set_speed(self, speed: float) -> None:
        """
        Set servo speed without time limit (continuous rotation).

        Args:
            speed: Speed value from -1.0 (full reverse) to 1.0 (full forward), 0.0 (stop)
        """
        if not -1.5 <= speed <= 1.5:
            raise ValueError("Speed must be between -1.5 and 1.5")

        with self._pin_lock:
            pulse_width = self.stop_duty + int(speed * 500)  # 1000us to 2000us range
            self._set_pwm(pulse_width)  # No duration - continuous rotation
            logger.debug(f"Set servo {self.pin} speed to {speed}")

    def rotate(self, angle: float) -> None:
        """
        Rotate the servo by a specific angle.

        Args:
            angle: Rotation angle in degrees (positive for forward, negative for reverse)
        """
        if angle == 0:
            return

        with self._pin_lock:
            # Calculate speed and time
            speed, time_360 = self._calculate_speed_and_time(abs(angle))
            time_to_angle = abs(angle) * time_360 / 360

            # Determine direction
            direction = 1 if angle > 0 else -1
            pulse_width = self.stop_duty + direction * int(speed * 500)

            # Execute rotation with duration
            logger.debug(f"Rotating servo {self.pin} by {angle}° at speed {speed} for {time_to_angle}s")
            self._set_pwm(pulse_width, duration=time_to_angle)
            logger.debug(f"Stopped servo {self.pin} after rotation")

    def stop(self) -> None:
        """Stop the servo motor."""
        with self._pin_lock:
            self._set_pwm_stop()
            logger.debug(f"Stopped servo {self.pin}")

    def cleanup(self) -> None:
        """Clean up GPIO resources for this servo."""
        if self._is_initialized:
            with self._pin_lock:
                self._set_pwm_stop()
                self._pin_usage[self.pin] = False
                self._is_initialized = False
                logger.info(f"Cleaned up servo on pin {self.pin}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.cleanup()

    def __del__(self):
        """Destructor to ensure cleanup."""
        try:
            self.cleanup()
        except:
            pass

    @classmethod
    def cleanup_all(cls) -> None:
        """
        Clean up all pin usage tracking.
        """
        cls._pin_usage.clear()
        logger.info("Cleaned up all servo resources")


class ServoController:
    """
    High-level controller for managing multiple servo motors.
    """

    def __init__(self):
        """Initialize the servo controller."""
        self.servos: Dict[str, ServoMotor] = {}

    def add_servo(self, name: str, pin: int, **kwargs) -> ServoMotor:
        """
        Add a new servo to the controller.

        Args:
            name: Unique name for the servo
            pin: GPIO pin number
            **kwargs: Additional arguments for ServoMotor constructor

        Returns:
            The created ServoMotor instance

        Raises:
            ValueError: If a servo with the same name already exists
        """
        if name in self.servos:
            raise ValueError(f"Servo with name '{name}' already exists")

        servo = ServoMotor(pin, **kwargs)
        self.servos[name] = servo
        return servo

    def get_servo(self, name: str) -> Optional[ServoMotor]:
        """Get a servo by name."""
        return self.servos.get(name)

    def remove_servo(self, name: str) -> None:
        """Remove and cleanup a servo."""
        if name in self.servos:
            self.servos[name].cleanup()
            del self.servos[name]

    def rotate_all(self, angle: float) -> None:
        """Rotate all servos by the same angle."""
        threads = []
        for servo in self.servos.values():
            thread = threading.Thread(target=servo.rotate, args=(angle,))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()

    def stop_all(self) -> None:
        """Stop all servos."""
        for servo in self.servos.values():
            servo.stop()

    def cleanup(self) -> None:
        """Clean up all servos and GPIO resources."""
        for servo in self.servos.values():
            servo.cleanup()
        self.servos.clear()
        ServoMotor.cleanup_all()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.cleanup()


# Example usage
if __name__ == "__main__":
    # Example 1: Using individual servo
    try:
        servo1 = ServoMotor(pin=18)
        servo1.rotate(20)
        servo1.rotate(-40)
        servo1.cleanup()
    except ServoMotorError as e:
        logger.error(f"Servo error: {e}")

    # Example 2: Using context manager
    with ServoMotor(pin=18) as servo:
        servo.rotate(90)
        time.sleep(0.5)
        servo.set_speed(0.5)
        time.sleep(1)
        servo.stop()

    # Example 3: Using ServoController for multiple servos
    with ServoController() as controller:
        controller.add_servo("left", pin=17)
        controller.add_servo("right", pin=18)

        # Rotate both servos simultaneously
        controller.rotate_all(45)

        # Control individual servos
        left_servo = controller.get_servo("left")
        if left_servo:
            left_servo.rotate(-30)

    # Ensure all resources are cleaned up
    ServoMotor.cleanup_all()
