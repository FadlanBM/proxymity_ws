#!/usr/bin/env python3
"""
Jetson Orin Nano platform detection and camera utilities.

Auto-detects whether this code is running on a Jetson (aarch64 / Tegra)
and provides the appropriate GStreamer pipeline for CSI cameras.
"""

import os
import platform
import subprocess
import contextlib


def is_jetson() -> bool:
    """Return True if running on a Jetson (aarch64 + Tegra)."""
    if platform.machine() != 'aarch64':
        return False
    # Check for NVIDIA Tegra / Jetson indicator in device-tree or /proc
    if os.path.exists('/sys/devices/soc0/family'):
        with open('/sys/devices/soc0/family') as f:
            if 'tegra' in f.read().lower():
                return True
    # Fallback: check for nvpmodel or tegra-revision
    if os.path.exists('/etc/nvpmodel.conf'):
        return True
    try:
        proc = subprocess.run(
            ['which', 'tegrastats'], capture_output=True, text=True, timeout=2
        )
        if proc.returncode == 0:
            return True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return False


def csi_gstreamer_pipeline(
    sensor_id: int = 0,
    sensor_mode: int = 3,
    capture_width: int = 1280,
    capture_height: int = 720,
    framerate: int = 30,
    flip_method: int = 0,
) -> str:
    """
    Return a GStreamer pipeline string for Jetson CSI cameras.
    """
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! '
        f'video/x-raw(memory:NVMM), '
        f'width={capture_width}, height={capture_height}, '
        f'framerate={framerate}/1 ! '
        f'nvvidconv flip-method={flip_method} ! '
        f'video/x-raw, width={capture_width}, height={capture_height}, '
        f'format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
    )


def open_camera(camera_index: int = 0, csi_sensor_id: int = 0, use_csi: bool = True) -> 'cv2.VideoCapture':
    """
    Open a camera with platform-appropriate pipeline.

    On Jetson this uses a GStreamer CSI pipeline if use_csi is True; otherwise,
    or on x86, it falls back to the standard cv2.VideoCapture(camera_index).
    Returns a cv2.VideoCapture handle.
    """
    import cv2

    if is_jetson() and use_csi:
        pipeline = csi_gstreamer_pipeline(sensor_id=csi_sensor_id)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            # Fallback to generic V4L2 on Jetson (e.g. USB cameras via V4L2)
            cap = cv2.VideoCapture(camera_index)
        return cap

    # Standard desktop / x86 / USB camera path
    cap = cv2.VideoCapture(camera_index)
    return cap


@contextlib.contextmanager
def suppress_stderr():
    """Context manager to suppress C-level stderr (fd 2) output."""
    try:
        original_stderr_fd = os.dup(2)
        devnull = os.open(os.devnull, os.O_WRONLY)
        try:
            os.dup2(devnull, 2)
            yield
        finally:
            os.dup2(original_stderr_fd, 2)
            os.close(original_stderr_fd)
            os.close(devnull)
    except Exception:
        yield
