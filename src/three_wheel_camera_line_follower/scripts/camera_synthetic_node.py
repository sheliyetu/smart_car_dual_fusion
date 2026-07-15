#!/usr/bin/env python3
"""Hybrid synthetic camera + OpenCV line detector for environments without GPU rendering.
Generates synthetic downward camera view from track geometry, then applies
OpenCV threshold → centroid → error (same logic as real camera detector).
"""
import math, os, sys

import cv2
import numpy as np
import rospy
import tf.transformations as tft
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Float32

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from track_utils import generate_track

IMG_SIZE = 64
CAM_FOV_M = 0.40  # increased from 0.20 for better curve detection
PIX_PER_M = 125
CANVAS_SIZE = 1000


class SyntheticDetector:
    def __init__(self):
        rospy.init_node("camera_synthetic_detector")

        # Build track canvas
        self.track = generate_track(samples_per_seg=12)
        self.canvas = np.ones((CANVAS_SIZE, CANVAS_SIZE), dtype=np.uint8) * 255
        pts = [[int(x * PIX_PER_M + CANVAS_SIZE / 2),
                int(-y * PIX_PER_M + CANVAS_SIZE / 2)] for x, y in self.track]
        cv2.polylines(self.canvas, [np.array(pts)], True, 0,
                      thickness=max(2, int(0.10 * PIX_PER_M)))
        self.canvas = cv2.GaussianBlur(self.canvas, (3, 3), 0.5)

        self.half_crop = int(CAM_FOV_M * PIX_PER_M) // 2
        self.loop_rate = float(rospy.get_param("~loop_rate", 30.0))

        self.error_pub = rospy.Publisher("/line_follower/error", Float32, queue_size=5)
        self.online_pub = rospy.Publisher("/line_follower/online", Bool, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self.timer_cb)
        rospy.loginfo("Synthetic detector started")

    def _camera_crop(self, cx, cy, yaw):
        px = int(cx * PIX_PER_M + CANVAS_SIZE / 2)
        py = int(-cy * PIX_PER_M + CANVAS_SIZE / 2)
        half = self.half_crop
        x1 = max(0, px - half)
        y1 = max(0, py - half)
        x2 = min(CANVAS_SIZE, px + half)
        y2 = min(CANVAS_SIZE, py + half)
        crop = np.ones((half * 2, half * 2), dtype=np.uint8) * 255
        ch, cw = y2 - y1, x2 - x1
        crop[0:ch, 0:cw] = self.canvas[y1:y2, x1:x2]
        M = cv2.getRotationMatrix2D((half, half), -math.degrees(yaw), 1.0)
        crop = cv2.warpAffine(crop, M, (half * 2, half * 2),
                              borderValue=255, flags=cv2.INTER_NEAREST)
        crop = cv2.resize(crop, (IMG_SIZE, IMG_SIZE), cv2.INTER_AREA)
        return crop

    def _detect_line(self, img):
        """OpenCV: threshold → morphology → weighted centroid → normalized error.
        Uses higher weight for bottom rows (near car) to detect curves early."""
        if img.mean() > 248:
            return 0.0
        _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        h, w = binary.shape
        ys, xs = np.nonzero(binary)
        if len(xs) < 20:
            return 0.0

        # Weighted centroid: bottom rows (higher y) have more weight
        weights = 0.25 + 0.75 * (ys.astype(np.float32) / float(max(1, h - 1)))
        cx = float(np.average(xs.astype(np.float32), weights=weights))

        error = (cx - w / 2) / (w / 2) * 2.0
        return max(-2.0, min(2.0, error))

    def _respawn(self):
        ms = ModelState()
        ms.model_name = "three_wheel_camera_car"
        ms.pose.position.x = 2.136986
        ms.pose.position.y = -0.207531
        ms.pose.position.z = 0.08
        q = tft.quaternion_from_euler(0, 0, 1.232498)
        ms.pose.orientation = Quaternion(*q)
        ms.reference_frame = "world"
        try:
            rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)(ms)
        except Exception:
            pass

    def timer_cb(self, _event):
        try:
            get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp = get_state("three_wheel_camera_car", "world")
            cx, cy, cz = resp.pose.position.x, resp.pose.position.y, resp.pose.position.z
            q = resp.pose.orientation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception:
            return
        if cz < -0.5:
            self._respawn()
            return
        try:
            img = self._camera_crop(cx, cy, yaw)
            error = self._detect_line(img)
            online = True  # synthetic image always has line data
            self.error_pub.publish(Float32(data=error))
            self.online_pub.publish(Bool(data=online))
        except Exception:
            pass


if __name__ == "__main__":
    try:
        SyntheticDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
