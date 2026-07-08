#!/usr/bin/env python3
"""
Green Light Detector for Spearhead/Proximity Gripper Release.

Detects a single point light source — such as a green laser pointer or
green LED — using HSV colour-space segmentation.  Among all green blobs
in the frame, only the one with the HIGHEST AVERAGE BRIGHTNESS (V channel)
is returned.  This ensures only a truly bright light source is tracked,
rather than reflections or dim green objects.

HSV ranges are exposed so they can be tuned at runtime via ROS parameters.
"""

import cv2
import numpy as np


class GreenLightDetector:
    """HSV-based single-point green light detector — picks the BRIGHTEST blob."""

    def __init__(
        self,
        hue_low: int = 40,
        hue_high: int = 80,
        sat_low: int = 50,
        sat_high: int = 255,
        val_low: int = 200,        # high threshold — only bright light passes
        val_high: int = 255,
        min_area: int = 3,         # laser dots can be just a few pixels
        kernel_size: int = 3,      # small kernel preserves fine detail
    ):
        self.lower = np.array([hue_low, sat_low, val_low], dtype=np.uint8)
        self.upper = np.array([hue_high, sat_high, val_high], dtype=np.uint8)
        self.min_area = min_area
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)

    def update_hsv_range(self, lower: tuple, upper: tuple) -> None:
        """Update HSV thresholds at runtime (useful for dynamic tuning)."""
        self.lower[:] = lower
        self.upper[:] = upper

    def detect(self, frame: np.ndarray) -> tuple[list, np.ndarray]:
        """
        Run detection on a BGR frame.

        Returns a SINGLE detection — the green blob with the HIGHEST
        average brightness (V channel).  This is ideal for following a
        laser dot or a single bright LED while ignoring dim reflections.

        Returns
        -------
        detections : list of (x, y, w, h, area, avg_brightness)
            List containing *at most one* item — the brightest green light.
            Empty list if no green light is found.
        mask : np.ndarray
            Binary mask after HSV thresholding + morphology.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # Clean up noise — small kernel preserves tiny laser dots
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        best_detection = None
        best_brightness = 0

        # Extract V channel for brightness scoring
        v_channel = hsv[:, :, 2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            # Compute average brightness (V) of this contour's pixels
            cnt_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
            bright_vals = v_channel[cnt_mask == 255]
            avg_brightness = float(np.mean(bright_vals)) if len(bright_vals) > 0 else 0

            # Keep the blob with the highest average brightness
            if avg_brightness > best_brightness:
                best_brightness = avg_brightness
                x, y, w, h = cv2.boundingRect(cnt)
                best_detection = (x, y, w, h, area, avg_brightness)

        detections = [best_detection] if best_detection is not None else []
        return detections, mask
