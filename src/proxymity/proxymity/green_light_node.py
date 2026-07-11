#!/usr/bin/env python3
"""
Green Light Detection Node for Proximity/Spearhead Gripper Release.

Waits for a trigger (via /fsm/area_command) to start detecting a green
light / green marker in the camera frame.  When the green light is consistently
present for N consecutive frames, it sends a gripper-open command ('O') to the Teensy
and publishes 'area_complete' on /fsm/signal.

Supports both:
  1. RealSense camera (subscribing to ROS 2 image topic)
  2. USB / CSI camera (direct OpenCV capture)
"""

import json
import os
import time

import cv2
import numpy as np
import rclpy
import serial
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from proxymity.green_light_detector import GreenLightDetector
from proxymity.jetson_utils import is_jetson, open_camera, suppress_stderr


class GreenLightNode(Node):
    def __init__(self):
        super().__init__('green_light_node')

        # Platform detection
        self.on_jetson = is_jetson()
        if self.on_jetson:
            self.get_logger().info('Green light node running on Jetson — headless/CSI optimised')

        # ---------- parameters ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('csi_sensor_id', 0)
        
        # Declare boolean parameters with dynamic typing to accept both bool and string
        bool_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('use_csi', False, descriptor=bool_desc)
        self.declare_parameter('use_realsense', True, descriptor=bool_desc)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')

        # HSV thresholds for green
        self.declare_parameter('hue_low', 50)           # Narrowed from 40 to ignore yellow-green noise
        self.declare_parameter('hue_high', 75)          # Narrowed from 80 to ignore blue-green noise
        self.declare_parameter('sat_low', 150)          # Increased from 50 to 150 to require pure green
        self.declare_parameter('sat_high', 255)
        self.declare_parameter('val_low', 225)          # Increased from 200 to 225 to require extreme brightness
        self.declare_parameter('val_high', 255)

        # Detection tuning
        self.declare_parameter('min_area', 5)           # Increased from 3 to 5 px minimum area
        self.declare_parameter('kernel_size', 3)        # morphology kernel
        self.declare_parameter('confidence_frames', 15)  # Increased from 5 to 15 (half second at 30fps)
        self.declare_parameter('max_missed_frames', 10)  # reset if lost for this many
        self.declare_parameter('timer_period_sec', 0.05)
        self.declare_parameter('show_debug_window', not self.on_jetson, descriptor=bool_desc)
        self.declare_parameter('debug_area', 'GREEN_LIGHT')

        # ---------- resolve values ----------
        self.serial_port_str = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.csi_sensor_id = int(self.get_parameter('csi_sensor_id').value)
        self.use_csi = self._get_bool_param('use_csi')
        self.use_realsense = self._get_bool_param('use_realsense')
        self.color_topic = self.get_parameter('color_topic').value

        hue_low = int(self.get_parameter('hue_low').value)
        hue_high = int(self.get_parameter('hue_high').value)
        sat_low = int(self.get_parameter('sat_low').value)
        sat_high = int(self.get_parameter('sat_high').value)
        val_low = int(self.get_parameter('val_low').value)
        val_high = int(self.get_parameter('val_high').value)
        min_area = int(self.get_parameter('min_area').value)
        kernel_size = int(self.get_parameter('kernel_size').value)
        self.confidence_frames = int(self.get_parameter('confidence_frames').value)
        self.max_missed_frames = int(self.get_parameter('max_missed_frames').value)
        timer_period = float(self.get_parameter('timer_period_sec').value)
        self.show_debug_window = self._get_bool_param('show_debug_window')
        self.debug_area = self.get_parameter('debug_area').value

        # ---------- detector ----------
        self.detector = GreenLightDetector(
            hue_low=hue_low,
            hue_high=hue_high,
            sat_low=sat_low,
            sat_high=sat_high,
            val_low=val_low,
            val_high=val_high,
            min_area=min_area,
            kernel_size=kernel_size,
        )

        # ---------- hardware ----------
        self.teensy = None
        self.cap = None
        self._init_serial()

        # ---------- state ----------
        self.is_detecting = False
        self.consecutive_green = 0
        self.consecutive_missed = 0
        self.green_locked = False  # once true, don't re-trigger

        self.bridge = CvBridge()
        # Handle OpenCV version cvtype_to_name overrides if needed
        self.bridge.cvtype_to_name[16] = '8UC3'
        self.bridge.cvtype_to_name[24] = '8UC4'

        # ---------- FSM integration ----------
        self.create_subscription(String, '/fsm/area_command', self._on_area_command, 10)
        self.fsm_signal_pub = self.create_publisher(String, '/fsm/signal', 10)
        self.area_status_pub = self.create_publisher(String, '/fsm/area_status', 10)
        self.fsm_command_pub = self.create_publisher(Int32, '/fsm_command', 10)

        # ---------- Camera / Topic Subscription setup ----------
        if self.use_realsense:
            self.get_logger().info(f'use_realsense is True: subscribing to color topic: {self.color_topic}')
            self.image_sub = self.create_subscription(
                Image,
                self.color_topic,
                self.realsense_image_cb,
                qos_profile=qos_profile_sensor_data
            )
        else:
            self.get_logger().info('use_realsense is False: using OpenCV VideoCapture.')
            if self.show_debug_window:
                # Debug/tuning mode: initialize camera immediately
                self._init_camera()

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Green light detection node initialized successfully.')
        
        # Start detection immediately, camera is always on
        self._start_detection()

    def _get_bool_param(self, name):
        val = self.get_parameter(name).value
        if isinstance(val, bool):
            return val
        if isinstance(val, str):
            return val.lower() in ('true', '1', 'yes', 'on')
        return bool(val)

    def _probe_openable_camera_indices(self, max_index=20):
        openable_indices = []
        with suppress_stderr():
            for index in range(max_index):
                if not os.path.exists(f'/dev/video{index}'):
                    continue
                probe = cv2.VideoCapture(index)
                try:
                    if probe.isOpened():
                        ret, frame = probe.read()
                        if ret and frame is not None and len(frame.shape) == 3 and frame.shape[2] == 3:
                            openable_indices.append((index, frame.shape[1]))
                finally:
                    probe.release()
        # Sort found indices by frame width in descending order to prefer standard color streams
        openable_indices.sort(key=lambda x: x[1], reverse=True)
        return [idx for idx, _ in openable_indices]

    # ------------------------------------------------------------------
    # Init helpers
    # ------------------------------------------------------------------
    def _init_serial(self):
        try:
            self.teensy = serial.Serial(
                self.serial_port_str, self.baud_rate, timeout=0.01
            )
            self.get_logger().info(f'Teensy connected on {self.serial_port_str}')
        except Exception as exc:
            self.teensy = None
            self.get_logger().warning(
                f'Failed to open serial port {self.serial_port_str}: {exc}. '
                'Commands will only be logged.'
            )

    def _init_camera(self):
        if self.on_jetson:
            with suppress_stderr():
                self.cap = open_camera(
                    camera_index=self.camera_index,
                    csi_sensor_id=self.csi_sensor_id,
                    use_csi=self.use_csi,
                )
        else:
            with suppress_stderr():
                self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().warn(f'Failed to open camera index {self.camera_index}. Probing for other openable cameras...')
            openable_indices = self._probe_openable_camera_indices()
            if openable_indices:
                fallback_index = openable_indices[0]
                self.get_logger().info(f'Falling back to first openable camera index: {fallback_index}')
                self.camera_index = fallback_index
                if self.on_jetson:
                    with suppress_stderr():
                        # The fallback camera is a probed USB/V4L2 device, so force use_csi=False
                        self.cap = open_camera(
                            camera_index=self.camera_index,
                            csi_sensor_id=self.csi_sensor_id,
                            use_csi=False,
                        )
                else:
                    with suppress_stderr():
                        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            raise RuntimeError(
                f'Failed to open camera index {self.camera_index} for green light detection.'
            )
        self.get_logger().info(f'Green light camera opened (index {self.camera_index}).')

    # ------------------------------------------------------------------
    # FSM callback
    # ------------------------------------------------------------------
    def _on_area_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Ignoring malformed area_command: {msg.data}')
            return

        if cmd.get('command') != 'start':
            return

        # Only respond to commands specifically for green detection.
        if cmd.get('task') != 'green_detection':
            return

        self.get_logger().info(
            f'Received start command for area "{cmd.get("area")}" via /fsm/area_command; '
            'starting green light detection'
        )
        self._start_detection()

    def _start_detection(self):
        if self.green_locked:
            self.get_logger().info('Green light already locked; ignoring duplicate start.')
            return
        if not self.use_realsense:
            if self.cap is None:
                self._init_camera()
        self.is_detecting = True
        self.consecutive_green = 0
        self.consecutive_missed = 0
        self.get_logger().info('Green light detection active.')

    # ------------------------------------------------------------------
    # Serial command
    # ------------------------------------------------------------------
    def send_cmd(self, cmd_str: str):
        self.get_logger().info(f'[GREEN_CMD] {cmd_str}')
        if self.teensy:
            try:
                full_cmd = f'{cmd_str}\n'
                self.teensy.write(full_cmd.encode('utf-8'))
            except serial.SerialException as exc:
                self.get_logger().warning(f'Failed to write to Teensy: {exc}. Closing serial port.')
                try:
                    self.teensy.close()
                except Exception:
                    pass
                self.teensy = None

    def _check_serial_reconnect(self):
        if not self.teensy:
            now = self.get_clock().now().nanoseconds / 1e9
            if not hasattr(self, '_last_reconnect_attempt'):
                self._last_reconnect_attempt = 0
            if now - self._last_reconnect_attempt > 5.0:
                self._last_reconnect_attempt = now
                self.get_logger().info('Attempting to reconnect to Teensy...')
                self._init_serial()

    # ------------------------------------------------------------------
    # RealSense Color Callback
    # ------------------------------------------------------------------
    def realsense_image_cb(self, msg: Image):
        if not self.is_detecting or self.green_locked:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        self._process_frame(frame)

    # ------------------------------------------------------------------
    # Common Frame Processing & State Machine
    # ------------------------------------------------------------------
    def _process_frame(self, frame: np.ndarray):
        detections, mask = self.detector.detect(frame)
        green_present = len(detections) > 0

        # ---------- state machine for green confirmation ----------
        if green_present:
            self.consecutive_green += 1
            self.consecutive_missed = 0
            if self.consecutive_green >= self.confidence_frames:
                self.get_logger().info(
                    f'Green light detected for {self.consecutive_green} frames — '
                    'publishing 40 to /fsm_command!'
                )
                fsm_msg = Int32()
                fsm_msg.data = 40
                self.fsm_command_pub.publish(fsm_msg)
                self.green_locked = True
                self.is_detecting = False
                if not self.use_realsense:
                    if self.cap:
                        self.cap.release()
                        self.cap = None
                        self.get_logger().info('OpenCV camera released (detection complete).')
                if self.show_debug_window:
                    cv2.destroyAllWindows()
                    self.get_logger().info('Closed green light preview windows.')
                self.fsm_signal_pub.publish(String(data='area_complete'))
                self.area_status_pub.publish(String(data=json.dumps({
                    "area": "AREA_1",
                    "task": "green_detection",
                    "status": "complete",
                })))
        else:
            self.consecutive_missed += 1
            if self.consecutive_missed >= self.max_missed_frames:
                self.consecutive_green = 0
                self.consecutive_missed = 0
                self.get_logger().info(
                    'Green light lost; resetting confidence counter.'
                )

        # ---------- debug overlay ----------
        if self.show_debug_window and self.is_detecting and not self.green_locked:
            for (x, y, w, h, area, brightness) in detections:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f'{area:.0f}',
                    (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

            status = 'DETECTING'
            cv2.putText(
                frame,
                f'Green: {status} ({self.consecutive_green}/{self.confidence_frames})',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow('Green Light Detection', frame)
            cv2.imshow('Green Light Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Shutting down on keyboard request')
                rclpy.shutdown()
            elif key == ord('d'):
                # manual toggle for testing
                if not self.green_locked:
                    self._start_detection()

    # ------------------------------------------------------------------
    # Timer callback (used for serial reconnect & OpenCV capture loop)
    # ------------------------------------------------------------------
    def timer_callback(self):
        self._check_serial_reconnect()

        # If using realsense, the image callback handles processing.
        # Otherwise, we capture from the webcam if detection is active.
        if self.use_realsense:
            return

        if not self.is_detecting or self.green_locked:
            return  # idle: not detecting green light

        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame from green-light camera')
            return

        self._process_frame(frame)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.teensy:
            self.teensy.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GreenLightNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
