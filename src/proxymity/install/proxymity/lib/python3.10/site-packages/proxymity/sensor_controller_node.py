"""
sensor_controller_node — ROS 2 node that controls robot movement based on proximity sensor state.

Behavior:
  - Subscribes to `/proximity/obstacle` (std_msgs/Bool).
  - When the sensor returns False (no obstacle), the robot waits for 1 second and then moves to the right using `/cmd_vel` until the sensor returns True.
  - When the sensor returns True (obstacle detected):
    - Waits for 1 second.
    - Publishes FSM command 31 to `/fsm_command` (std_msgs/Int32) once.
    - Publishes a backward command via `/cmd_vel` for a specified duration.
    - Stays idle (zero velocity) if the sensor remains True after the backward command.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


class ProximityControllerNode(Node):
    """ROS 2 node that uses the proximity sensor state to trigger backward motion / right movement with delays."""

    def __init__(self):
        super().__init__('proximity_controller_node')

        # ---------- Parameters ----------
        self.declare_parameter('sensor_topic', '/proximity/obstacle')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('fsm_command_topic', '/fsm_command')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('fsm_command_val', 31)
        self.declare_parameter('backward_duration', 1.0)
        self.declare_parameter('action_delay_duration', 1.0)

        # Right movement velocities (default: strafe right via negative Y velocity)
        self.declare_parameter('right_linear_x', 0.0)
        self.declare_parameter('right_linear_y', -0.08)
        self.declare_parameter('right_angular_z', 0.0)

        # Backward movement velocities
        self.declare_parameter('backward_linear_x', -0.1)
        self.declare_parameter('backward_linear_y', 0.0)
        self.declare_parameter('backward_angular_z', 0.0)

        # ---------- Retrieve Parameters ----------
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.fsm_command_topic = self.get_parameter('fsm_command_topic').value

        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.fsm_command_val = self.get_parameter('fsm_command_val').value
        self.backward_duration = self.get_parameter('backward_duration').value
        self.action_delay_duration = self.get_parameter('action_delay_duration').value

        # Configure right velocity Twist
        self.right_vel = Twist()
        self.right_vel.linear.x = self.get_parameter('right_linear_x').value
        self.right_vel.linear.y = self.get_parameter('right_linear_y').value
        self.right_vel.angular.z = self.get_parameter('right_angular_z').value

        # Configure backward velocity Twist
        self.backward_vel = Twist()
        self.backward_vel.linear.x = self.get_parameter('backward_linear_x').value
        self.backward_vel.linear.y = self.get_parameter('backward_linear_y').value
        self.backward_vel.angular.z = self.get_parameter('backward_angular_z').value

        # ---------- State Machine Setup ----------
        self.STATE_UNKNOWN = 'UNKNOWN'
        self.STATE_DELAYING_TO_RIGHT = 'DELAYING_TO_RIGHT'
        self.STATE_MOVING_RIGHT = 'MOVING_RIGHT'
        self.STATE_MOVING_BACKWARD = 'MOVING_BACKWARD'
        self.STATE_IDLE_OBSTACLE = 'IDLE_OBSTACLE'

        self.state = self.STATE_UNKNOWN
        self.last_sensor_value = None
        self.delay_start_time = None
        self.backward_start_time = None

        # Gate: sensor is ignored until FSM 30 has been sent and confirmed
        self.armed = False

        # ---------- Publishers & Subscribers ----------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.fsm_pub = self.create_publisher(Int32, self.fsm_command_topic, 10)

        self.sensor_sub = self.create_subscription(
            Bool,
            self.sensor_topic,
            self.sensor_callback,
            10
        )

        # ---------- Control Timer ----------
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        # ---------- Publish FSM startup command (30) ----------
        startup_msg = Int32()
        startup_msg.data = 30
        self.fsm_pub.publish(startup_msg)
        self.get_logger().info("Published FSM startup command 30. Sensor will arm in 0.5s.")

        # One-shot timer: arm the sensor after FSM 30 has had time to propagate
        self.arm_timer = self.create_timer(5, self._arm_sensor)

        self.get_logger().info(
            f"Proximity Controller Node started:\n"
            f"  Subscribed to: '{self.sensor_topic}'\n"
            f"  Publishing Twist to: '{self.cmd_vel_topic}'\n"
            f"  Publishing FSM to: '{self.fsm_command_topic}'\n"
            f"  Initial State: {self.state} (sensor DISARMED)"
        )

    def _arm_sensor(self) -> None:
        """One-shot callback: arms the sensor after FSM 30 has propagated."""
        self.armed = True
        self.arm_timer.cancel()
        self.get_logger().info("Sensor ARMED — now responding to proximity events.")

    def sensor_callback(self, msg: Bool) -> None:
        """Process incoming sensor readings to update internal state transitions."""
        if not self.armed:
            return  # Ignore all sensor readings until FSM 30 is complete

        sensor_val = msg.data
        self.last_sensor_value = sensor_val
        now = self.get_clock().now()

        if sensor_val is True:
            if self.state in (self.STATE_UNKNOWN, self.STATE_MOVING_RIGHT, self.STATE_DELAYING_TO_RIGHT):
                # Interrupt and start moving backward; FSM command will fire after backward completes
                self.state = self.STATE_MOVING_BACKWARD
                self.backward_start_time = now
                self.get_logger().info("Sensor returned True (obstacle detected)! Moving backward.")
                self.cmd_vel_pub.publish(self.backward_vel)
        else:
            if self.state in (self.STATE_UNKNOWN, self.STATE_IDLE_OBSTACLE):
                self.state = self.STATE_DELAYING_TO_RIGHT
                self.delay_start_time = now
                self.get_logger().info(
                    f"Sensor returned False (clear). Entering {self.action_delay_duration}s delay "
                    "before moving right."
                )

    def timer_callback(self) -> None:
        """Run periodic velocity publisher based on current state."""
        if self.state == self.STATE_UNKNOWN:
            # Waiting for the first sensor measurement
            return

        elif self.state == self.STATE_DELAYING_TO_RIGHT:
            self.publish_zero_velocity()
            now = self.get_clock().now()
            elapsed = (now - self.delay_start_time).nanoseconds / 1e9
            if elapsed >= self.action_delay_duration:
                self.state = self.STATE_MOVING_RIGHT
                self.get_logger().info("Delay completed. Transitioning to MOVING_RIGHT.")
                self.cmd_vel_pub.publish(self.right_vel)

        elif self.state == self.STATE_MOVING_RIGHT:
            self.cmd_vel_pub.publish(self.right_vel)

        elif self.state == self.STATE_MOVING_BACKWARD:
            now = self.get_clock().now()
            elapsed = (now - self.backward_start_time).nanoseconds / 1e9

            if elapsed >= self.backward_duration:
                self.get_logger().info("Backward command duration completed. Publishing FSM command and shutting down.")
                self.publish_zero_velocity()
                # Publish FSM command 31 after backward finishes
                fsm_msg = Int32()
                fsm_msg.data = self.fsm_command_val
                self.fsm_pub.publish(fsm_msg)
                self.get_logger().info(f"Published FSM command {self.fsm_command_val} to {self.fsm_command_topic}")
                raise SystemExit(0)
            else:
                self.cmd_vel_pub.publish(self.backward_vel)

        elif self.state == self.STATE_IDLE_OBSTACLE:
            self.publish_zero_velocity()

    def publish_zero_velocity(self) -> None:
        """Publish zero Twist to stop all movement."""
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ProximityControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('Shutting down proximity controller node')
    finally:
        # Stop the robot on shutdown
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
