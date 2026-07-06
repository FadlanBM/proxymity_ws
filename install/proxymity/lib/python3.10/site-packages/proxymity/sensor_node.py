"""
proxymity_node — ROS 2 node for proximity sensor (HC-SR04).

Publishes:
  - /proximity/range  (sensor_msgs/Range) — standard ROS2 range message
  - /proximity/raw    (std_msgs/Float32)   — raw distance in cm

Subscribes:
  - ~enable           (std_msgs/Bool)      — enable/disable sensor reading (optional)
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from .hcsr04_driver import HCSR04Driver, HCSR04DriverSim


class ProximitySensorNode(Node):
    """ROS 2 node that reads a HC-SR04 proximity sensor and publishes range data."""

    def __init__(self):
        super().__init__('proximity_sensor_node')

        # ---------- Parameters ----------
        self.declare_parameter('trigger_pin', 11)
        self.declare_parameter('echo_pin', 12)
        self.declare_parameter('gpio_chip', '/dev/gpiochip0')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('simulate', False)
        self.declare_parameter('frame_id', 'proximity_link')
        self.declare_parameter('radiation_type', Range.ULTRASOUND)
        self.declare_parameter('field_of_view', 0.261799)   # ~15 degrees (radians)
        self.declare_parameter('min_range', 0.02)            # 2 cm
        self.declare_parameter('max_range', 4.0)             # 4 m

        trig_pin = self.get_parameter('trigger_pin').value
        echo_pin = self.get_parameter('echo_pin').value
        gpio_chip = self.get_parameter('gpio_chip').value
        rate = self.get_parameter('publish_rate_hz').value
        self._simulate = self.get_parameter('simulate').value

        # Range message static fields
        self._frame_id = self.get_parameter('frame_id').value
        self._radiation_type = self.get_parameter('radiation_type').value
        self._fov = self.get_parameter('field_of_view').value
        self._min_range = self.get_parameter('min_range').value
        self._max_range = self.get_parameter('max_range').value

        # ---------- Display pin info ----------
        self.get_logger().info(
            f'Proximity Sensor configured:\n'
            f'  Trigger GPIO line: {trig_pin}\n'
            f'  Echo GPIO line:    {echo_pin}\n'
            f'  GPIO chip:         {gpio_chip}\n'
            f'  Rate:              {rate} Hz'
        )

        # ---------- Hardware / Sim ----------
        if self._simulate or self._detect_simulation():
            self._driver = HCSR04DriverSim()
            self.get_logger().warn('Running in SIMULATION mode')
        else:
            try:
                self._driver = HCSR04Driver(
                    trigger_line=trig_pin,
                    echo_line=echo_pin,
                    gpio_chip=gpio_chip,
                )
                self.get_logger().info('HC-SR04 driver initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize GPIO driver: {e}')
                self.get_logger().warn('Falling back to SIMULATION mode')
                self._driver = HCSR04DriverSim()

        # ---------- Publishers ----------
        self._pub_range = self.create_publisher(Range, '/proximity/range', 10)
        self._pub_raw = self.create_publisher(Float32, '/proximity/raw', 10)

        # ---------- Timer ----------
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'ProximitySensorNode started — publishing to /proximity/range '
            f'and /proximity/raw at {rate} Hz'
        )

    def _detect_simulation(self) -> bool:
        """Detect if we're on actual Jetson hardware."""
        try:
            model = open('/proc/device-tree/model', 'r').read().strip()
            if 'jetson' in model.lower() or 'tegra' in model.lower():
                return False
            return True
        except Exception:
            return True  # assume simulation if we can't read model

    def _timer_callback(self) -> None:
        """Read sensor and publish."""
        distance_cm = self._driver.measure()
        distance_m = distance_cm / 100.0 if distance_cm > 0 else float('inf')

        now = self.get_clock().now()

        # --- Range message ---
        msg = Range()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.radiation_type = self._radiation_type
        msg.field_of_view = self._fov
        msg.min_range = self._min_range
        msg.max_range = self._max_range

        if 0 <= distance_m <= self._max_range:
            msg.range = distance_m
        else:
            msg.range = float('inf')  # out of range

        self._pub_range.publish(msg)

        # --- Raw Float32 ---
        raw_msg = Float32()
        raw_msg.data = distance_cm if distance_cm > 0 else -1.0
        self._pub_raw.publish(raw_msg)

        self.get_logger().debug(
            f'Distance: {distance_cm:.1f} cm ({distance_m:.2f} m)',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self) -> bool:
        self._driver.cleanup()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down proximity sensor node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
