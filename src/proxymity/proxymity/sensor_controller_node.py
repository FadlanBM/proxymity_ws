"""
sensor_controller_node — ROS 2 node that controls robot movement based on proximity sensor state.

Behavior:
  - Subscribes to `/proximity/obstacle` (std_msgs/Bool).
  - When the node starts, the robot lowers spearhead (FSM 32) and arms the sensor after 5.0s.
  - Once armed, if the sensor is False (no obstacle), the robot moves to the right using `/relative_move_slow` (y: -0.5).
  - When the sensor returns True (obstacle detected):
    - Publishes block 0 (all zeros) to `/relative_move_slow` to stop Y movement.
    - Publishes a relative move command (x: -0.5) to `/relative_move_slow`.
    - Waits for forward_duration (3.0s).
    - Publishes FSM command 31 to `/fsm_command` (std_msgs/Int32) once.
    - Shuts down.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Vector3


class ProximityControllerNode(Node):
    """ROS 2 node that controls robot movement based on proximity sensor state."""

    def __init__(self):
        super().__init__('proximity_controller_node')

        # ---------- Parameters ----------
        self.declare_parameter('sensor_topic', '/proximity/obstacle')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('relative_move_topic', '/relative_move_slow')
        self.declare_parameter('fsm_command_topic', '/fsm_command')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('fsm_command_val', 31)

        # Relative move distance parameters
        self.declare_parameter('initial_relative_y', -0.5)
        self.declare_parameter('forward_relative_x', -0.5)
        self.declare_parameter('forward_duration', 3.0)

        # Legacy parameters (kept for backward compatibility)
        self.declare_parameter('initial_linear_y', -0.05)
        self.declare_parameter('forward_linear_x', -0.2)
        self.declare_parameter('right_linear_y', -0.05)
        self.declare_parameter('backward_linear_x', -0.2)
        self.declare_parameter('backward_duration', 1.5)

        # ---------- Retrieve Parameters ----------
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.relative_move_topic = self.get_parameter('relative_move_topic').value
        self.fsm_command_topic = self.get_parameter('fsm_command_topic').value

        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.fsm_command_val = self.get_parameter('fsm_command_val').value

        self.initial_relative_y = self.get_parameter('initial_relative_y').value
        self.forward_relative_x = self.get_parameter('forward_relative_x').value
        self.forward_duration = self.get_parameter('forward_duration').value

        # ---------- State Machine Setup ----------
        self.STATE_START = 'START'
        self.STATE_MOVING_Y = 'MOVING_Y'
        self.STATE_MOVING_FORWARD = 'MOVING_FORWARD'
        self.STATE_FINISHED = 'FINISHED'

        self.state = self.STATE_START
        self.last_sensor_value = None
        self.sensor_history = []
        self.forward_start_time = None
        self.moving_y_start_time = None
        self.current_y_speed = self.initial_relative_y

        # Gate: sensor is ignored until FSM 32 has been sent and confirmed
        self.armed = False

        # ---------- Publishers & Subscribers ----------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.relative_move_pub = self.create_publisher(Vector3, self.relative_move_topic, 10)
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

        # One-shot timer: publish FSM 32 after 1.0s delay (allowing DDS discovery to settle)
        self.startup_timer = self.create_timer(1.0, self._lower_spearhead)

        self.get_logger().info(
            f"Proximity Controller Node started:\n"
            f"  Subscribed to: '{self.sensor_topic}'\n"
            f"  Publishing Vector3 to: '{self.relative_move_topic}'\n"
            f"  Publishing FSM to: '{self.fsm_command_topic}'\n"
            f"  Initial State: {self.state} (sensor DISARMED)"
        )

    def _lower_spearhead(self) -> None:
        """One-shot callback: publishes startup command 32 to lower spearhead, then starts 5s delay to arm."""
        self.startup_timer.cancel()

        # Publish FSM startup command (32) now that connections are established
        startup_msg = Int32()
        startup_msg.data = 32
        self.fsm_pub.publish(startup_msg)
        self.get_logger().info("Published FSM startup command 32 (lower spearhead). Waiting 5.0s before arming sensor...")

        # Start a 5.0s timer to arm the sensor after lowering is done
        self.arm_timer = self.create_timer(5.0, self._arm_sensor)

    def _arm_sensor(self) -> None:
        """One-shot callback: arms the sensor after 5.0s lowering delay."""
        self.armed = True
        self.arm_timer.cancel()
        self.get_logger().info("Sensor ARMED — now responding to proximity events.")

    def sensor_callback(self, msg: Bool) -> None:
        """Process incoming sensor readings to update internal state transitions."""
        if not self.armed:
            return  # Ignore all sensor readings until armed

        sensor_val = msg.data
        self.last_sensor_value = sensor_val

        if sensor_val is True:
            if self.state in (self.STATE_START, self.STATE_MOVING_Y):
                self.state = self.STATE_MOVING_FORWARD
                self.forward_start_time = self.get_clock().now()
                self.moving_y_start_time = None
                self.get_logger().info(f"Sensor message -> True (obstacle/target detected)! Stopping Y and moving X: {self.forward_relative_x} via /relative_move_slow.")
                
                # Send block 0 (x=0.0, y=0.0, z=0.0) to stop the robot
                stop_msg = Vector3()
                stop_msg.x = 0.0
                stop_msg.y = 0.0
                stop_msg.z = 0.0
                self.relative_move_pub.publish(stop_msg)

                # Send relative move x: -0.5
                forward_msg = Vector3()
                forward_msg.x = self.forward_relative_x
                forward_msg.y = 0.0
                forward_msg.z = 0.0
                self.relative_move_pub.publish(forward_msg)
        else:
            if self.state == self.STATE_START:
                self.state = self.STATE_MOVING_Y
                self.moving_y_start_time = self.get_clock().now()
                self.get_logger().info(f"Sensor message -> False (clear)! Changing movement to MOVING_Y (via /relative_move_slow y: {self.current_y_speed}).")
                
                # Send initial relative move
                rel_msg = Vector3()
                rel_msg.x = 0.0
                rel_msg.y = self.current_y_speed
                rel_msg.z = 0.0
                self.relative_move_pub.publish(rel_msg)

    def timer_callback(self) -> None:
        """Run periodic status check or keepalive."""
        now = self.get_clock().now()

        if self.state == self.STATE_MOVING_Y:
            # Send current Y relative move repeatedly
            rel_msg = Vector3()
            rel_msg.x = 0.0
            rel_msg.y = self.current_y_speed
            rel_msg.z = 0.0
            self.relative_move_pub.publish(rel_msg)

            if self.moving_y_start_time is not None:
                elapsed = (now - self.moving_y_start_time).nanoseconds / 1e9
                if elapsed >= 5.0:
                    self.get_logger().info("Y movement timeout (5.0s) reached. Publishing FSM command 0 and shutting down.")
                    # Send stop command to ensure we are stopped
                    stop_msg = Vector3()
                    stop_msg.x = 0.0
                    stop_msg.y = 0.0
                    stop_msg.z = 0.0
                    self.relative_move_pub.publish(stop_msg)

                    # Publish FSM command 0
                    fsm_msg = Int32()
                    fsm_msg.data = 0
                    self.fsm_pub.publish(fsm_msg)
                    self.get_logger().info(f"Published FSM command 0 to {self.fsm_command_topic}")
                    self.state = self.STATE_FINISHED
                    raise SystemExit(0)

        elif self.state == self.STATE_MOVING_FORWARD:
            # Send forward X relative move repeatedly
            forward_msg = Vector3()
            forward_msg.x = self.forward_relative_x
            forward_msg.y = 0.0
            forward_msg.z = 0.0
            self.relative_move_pub.publish(forward_msg)

            elapsed = (now - self.forward_start_time).nanoseconds / 1e9

            if elapsed >= self.forward_duration:
                self.get_logger().info("Forward/backward command duration completed. Publishing FSM command and shutting down.")
                # Send stop command to ensure we are stopped
                stop_msg = Vector3()
                stop_msg.x = 0.0
                stop_msg.y = 0.0
                stop_msg.z = 0.0
                self.relative_move_pub.publish(stop_msg)

                # Publish FSM command 31
                fsm_msg = Int32()
                fsm_msg.data = self.fsm_command_val
                self.fsm_pub.publish(fsm_msg)
                self.get_logger().info(f"Published FSM command {self.fsm_command_val} to {self.fsm_command_topic}")
                self.state = self.STATE_FINISHED
                raise SystemExit(0)

    def publish_zero_velocity(self) -> None:
        """Publish zero relative move and zero velocity Twist to stop all movement."""
        zero_rel = Vector3()
        self.relative_move_pub.publish(zero_rel)
        
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
