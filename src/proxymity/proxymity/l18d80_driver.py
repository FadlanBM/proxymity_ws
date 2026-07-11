"""
L18D80 IR Obstacle Sensor Driver for Jetson Orin Nano GPIO.

L18D80 is a digital infrared obstacle avoidance sensor.
- Output LOW  (0) = obstacle detected
- Output HIGH (1) = clear / no obstacle

Uses libgpiod v2 for GPIO control via /dev/gpiochip* devices.

Pin reference for Jetson Orin Nano 40-pin header:
  - Pin 9  = GND (Ground for sensor)
  - VCC    = 3.3V or 5V
  - OUT    = GPIO pin (default Pin 11 / GPIO11)
"""

import time

try:
    import gpiod
    HAS_GPIOD = True
except ImportError:
    HAS_GPIOD = False
    gpiod = None


class L18D80Driver:
    """Driver for L18D80 IR obstacle sensor using libgpiod."""

    def __init__(self, out_pin: int,
                 gpio_chip: str = '/dev/gpiochip0',
                 active_low: bool = True):
        """
        Args:
            out_pin:    GPIO line offset for the sensor OUT pin.
            gpio_chip:  GPIO chip device path (default: /dev/gpiochip0).
            active_low: If True, LOW means obstacle detected (L18D80 default).
                        If False, HIGH means obstacle detected.
        """
        if not HAS_GPIOD:
            raise ImportError(
                "Python 'gpiod' bindings not found. "
                "Install via: pip3 install gpiod"
            )

        self._out_pin = out_pin
        self._active_low = active_low
        self._chip = gpiod.Chip(gpio_chip)

        # Request OUT pin as input with Pull-Up Bias to prevent floating noise
        self._out = self._chip.request_lines(
            consumer='proxymity_l18d80',
            config={
                out_pin: gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    # bias=gpiod.line.Bias.PULL_UP,
                ),
            },
        )

    def is_obstacle_detected(self) -> bool:
        """
        Read the sensor state.

        Returns:
            True if obstacle is detected, False otherwise.
        """
        value = self._out.get_value(self._out_pin)
        if self._active_low:
            return value == gpiod.line.Value.INACTIVE   # LOW = obstacle
        else:
            return value == gpiod.line.Value.ACTIVE     # HIGH = obstacle

    def cleanup(self) -> None:
        """Release GPIO lines."""
        self._out.release()
        self._chip.close()


class L18D80DriverSim:
    """
    Simulated L18D80 driver for development/testing.

    Toggles obstacle state every ~2 seconds.
    """

    def __init__(self, *args, **kwargs):
        self._last_toggle = time.monotonic()
        self._state = False
        print("[L18D80DriverSim] SIMULATION MODE — not using real GPIO")

    def is_obstacle_detected(self) -> bool:
        now = time.monotonic()
        if now - self._last_toggle > 2.0:
            self._state = not self._state
            self._last_toggle = now
        return self._state

    def cleanup(self):
        print("[L18D80DriverSim] cleanup — nothing to do")
