#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Camera-based black-line detector for ROS1 Noetic.

Pipeline:
  /camera/image_raw -> ROI -> grayscale/blur -> black-line segmentation
  -> morphology -> largest contour/weighted centroid -> normalized lateral error.

Published topics:
  /line_follower/error       std_msgs/Float32, positive means line is to image right
  /line_follower/online      std_msgs/Bool
  /line_follower/center_x    std_msgs/Float32, pixel center in full image
  /line_follower/debug_image sensor_msgs/Image, annotated BGR image
  /line_follower/mask_image  sensor_msgs/Image, 8-bit mask

All important vision parameters are refreshed at runtime, so the tuner GUI can
adjust ROI, threshold mode and filtering without restarting ROS nodes.
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


def param(name, default):
    return rospy.get_param("~" + name, rospy.get_param("/" + name, default))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class CameraLineDetectorNode(object):
    def __init__(self):
        rospy.init_node("camera_line_detector_node")
        self.bridge = CvBridge()
        self.last_param_refresh = rospy.Time(0)
        self.image_topic = str(param("image_topic", "/camera/image_raw"))
        self.load_params(force=True)

        self.last_error = 0.0
        self.last_online = False
        self.last_stamp = rospy.Time.now()

        self.error_pub = rospy.Publisher("/line_follower/camera_error", Float32, queue_size=5)
        self.online_pub = rospy.Publisher("/line_follower/camera_online", Bool, queue_size=5)
        self.center_x_pub = rospy.Publisher("/line_follower/center_x", Float32, queue_size=5)
        self.status_pub = rospy.Publisher("/line_follower/vision_state", String, queue_size=5)
        self.debug_pub = rospy.Publisher("/line_follower/debug_image", Image, queue_size=2)
        self.mask_pub = rospy.Publisher("/line_follower/mask_image", Image, queue_size=2)

        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("camera_line_detector_node started, subscribe %s", self.image_topic)

    def load_params(self, force=False):
        now = rospy.Time.now()
        if not force and (now - self.last_param_refresh).to_sec() < 0.20:
            return
        self.last_param_refresh = now
        self.roi_top_ratio = float(param("roi_top_ratio", 0.55))
        self.roi_bottom_ratio = float(param("roi_bottom_ratio", 0.95))
        self.roi_left_ratio = float(param("roi_left_ratio", 0.05))
        self.roi_right_ratio = float(param("roi_right_ratio", 0.95))
        self.threshold_mode = str(param("threshold_mode", "otsu")).lower()  # otsu | fixed | adaptive
        self.black_threshold = int(param("black_threshold", 90))
        self.adaptive_block_size = int(param("adaptive_block_size", 31))
        self.adaptive_c = int(param("adaptive_c", 7))
        self.blur_kernel = int(param("blur_kernel", 5))
        self.morph_kernel = int(param("morph_kernel", 5))
        self.min_area = float(param("min_area", 180.0))
        self.min_black_ratio = float(param("min_black_ratio", 0.002))
        # 当前相机安装位置已经不会拍到自车大黑块，因此默认关闭自车遮挡过滤。
        # 遇到大弯道/宽黑线时，黑色区域可能很大，不能因为面积大就判丢线。
        self.self_mask_filter_enable = bool(param("self_mask_filter_enable", False))
        self.max_black_ratio = float(param("max_black_ratio", 1.0))
        self.max_contour_area_ratio = float(param("max_contour_area_ratio", 1.0))
        self.reject_round_blob = bool(param("reject_round_blob", False))
        self.round_blob_min_area_ratio = float(param("round_blob_min_area_ratio", 0.035))
        self.error_filter_alpha = float(param("error_filter_alpha", 0.60))
        self.use_weighted_centroid = bool(param("use_weighted_centroid", True))
        self.publish_mask = bool(param("publish_mask", True))

    @staticmethod
    def _odd_kernel(value, min_value=1, max_value=31):
        v = int(round(value))
        v = max(min_value, min(max_value, v))
        if v > 1 and v % 2 == 0:
            v += 1
        return v

    def make_mask(self, gray_roi: np.ndarray) -> np.ndarray:
        blur = self._odd_kernel(self.blur_kernel, 1, 31)
        if blur >= 3:
            gray_roi = cv2.GaussianBlur(gray_roi, (blur, blur), 0)

        if self.threshold_mode == "fixed":
            _, mask = cv2.threshold(gray_roi, int(clamp(self.black_threshold, 0, 255)), 255, cv2.THRESH_BINARY_INV)
        elif self.threshold_mode == "adaptive":
            block = self._odd_kernel(self.adaptive_block_size, 3, 99)
            c = int(self.adaptive_c)
            mask = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY_INV, block, c)
        else:
            _, mask = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        morph = self._odd_kernel(self.morph_kernel, 1, 31)
        if morph >= 3:
            k = np.ones((morph, morph), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def _select_line_contour(self, contours, mask_shape):
        """Select the most likely black track contour.

        Default behavior is deliberately simple: choose the largest valid black
        contour. The camera has been moved so it no longer sees the robot body;
        therefore large black regions in the ROI are usually real track curves,
        not obstacles. Optional self-mask filtering can still be enabled by
        setting /self_mask_filter_enable true.
        """
        h, w = mask_shape[:2]
        roi_area = float(max(1, h * w))
        best = None
        best_score = -1.0

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            bbox_area = float(max(1, bw * bh))
            fill_ratio = area / bbox_area
            aspect = max(float(bw) / float(max(1, bh)), float(bh) / float(max(1, bw)))
            area_ratio = area / roi_area
            touches_edge = (x <= 2) or (y <= 2) or (x + bw >= w - 3) or (y + bh >= h - 3)

            if self.self_mask_filter_enable:
                max_area_abs = self.max_contour_area_ratio * roi_area if self.max_contour_area_ratio > 0 else 1e18
                compact_round_blob = (
                    self.reject_round_blob
                    and area_ratio >= self.round_blob_min_area_ratio
                    and aspect < 1.45
                    and fill_ratio > 0.50
                    and not touches_edge
                )
                if compact_round_blob:
                    continue
                if area > max_area_abs and aspect < 1.8 and not touches_edge:
                    continue

            # Prefer contours near the bottom of the ROI because they represent
            # the line closest to the robot. This reduces left-right oscillation
            # on curves while still accepting large curved track segments.
            bottom_weight = 1.0 + 0.45 * ((y + bh) / float(max(1, h)))
            shape_weight = 1.0 + 0.15 * min(aspect, 5.0)
            score = area * bottom_weight * shape_weight
            if score > best_score:
                best_score = score
                best = contour
        return best

    def centroid_from_mask(self, mask: np.ndarray) -> Tuple[bool, Optional[float], Optional[float], float, Optional[np.ndarray]]:
        h, w = mask.shape[:2]
        black_pixels = int(cv2.countNonZero(mask))
        black_ratio = float(black_pixels) / float(max(1, h * w))
        if black_ratio < self.min_black_ratio:
            return False, None, None, 0.0, None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, 0.0, None

        contour = self._select_line_contour(contours, mask.shape)
        if contour is None:
            # 不做大面积误杀：只要存在轮廓，就回退到最大轮廓。
            contour = max(contours, key=cv2.contourArea)

        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return False, None, None, area, contour

        # 只在所选轮廓上求质心，避免 ROI 中其他黑色噪声影响误差。
        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        if self.use_weighted_centroid:
            ys, xs = np.nonzero(contour_mask)
            if len(xs) < 10:
                return False, None, None, area, contour
            # 越靠近图像底部，权重越大；这样遇到大弯时不会被远处弯道拖得左右猛晃。
            weights = 0.25 + 0.75 * (ys.astype(np.float32) / float(max(1, h - 1)))
            cx = float(np.average(xs.astype(np.float32), weights=weights))
            cy = float(np.average(ys.astype(np.float32), weights=weights))
        else:
            m = cv2.moments(contour)
            if abs(m["m00"]) < 1e-6:
                return False, None, None, area, contour
            cx = float(m["m10"] / m["m00"])
            cy = float(m["m01"] / m["m00"])
        return True, cx, cy, area, contour

    def image_cb(self, msg: Image):
        self.load_params(force=False)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        h, w = frame.shape[:2]
        y1 = int(clamp(self.roi_top_ratio, 0.0, 0.99) * h)
        y2 = int(clamp(self.roi_bottom_ratio, 0.01, 1.0) * h)
        x1 = int(clamp(self.roi_left_ratio, 0.0, 0.99) * w)
        x2 = int(clamp(self.roi_right_ratio, 0.01, 1.0) * w)
        if y2 <= y1 + 5 or x2 <= x1 + 5:
            y1, y2, x1, x2 = int(0.55 * h), int(0.95 * h), int(0.05 * w), int(0.95 * w)

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = self.make_mask(gray)
        online, cx_roi, cy_roi, area, contour = self.centroid_from_mask(mask)

        debug = frame.copy()
        cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 180, 0), 2)
        image_center_x = 0.5 * (x1 + x2)
        cv2.line(debug, (int(image_center_x), y1), (int(image_center_x), y2), (255, 0, 0), 1)

        if online and cx_roi is not None and cy_roi is not None:
            cx = x1 + cx_roi
            cy = y1 + cy_roi
            raw_error = (cx - image_center_x) / max(1.0, 0.5 * (x2 - x1))
            raw_error = clamp(raw_error, -1.0, 1.0)
            alpha = clamp(self.error_filter_alpha, 0.0, 0.98)
            error = alpha * self.last_error + (1.0 - alpha) * raw_error
            self.last_error = error
            self.last_online = True
            self.last_stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

            if contour is not None:
                c_shifted = contour + np.array([[[x1, y1]]], dtype=contour.dtype)
                cv2.drawContours(debug, [c_shifted], -1, (0, 255, 0), 2)
            cv2.circle(debug, (int(cx), int(cy)), 7, (0, 0, 255), -1)
            cv2.line(debug, (int(image_center_x), int(cy)), (int(cx), int(cy)), (0, 165, 255), 3)
            status = "online area={:.0f} error={:+.3f} mode={} ROI=({:.2f},{:.2f}) selfMask={}".format(
                area, error, self.threshold_mode, self.roi_top_ratio, self.roi_bottom_ratio, self.self_mask_filter_enable)
        else:
            error = self.last_error
            self.last_online = False
            status = "lost area={:.0f} keep_last_error={:+.3f} mode={} ROI=({:.2f},{:.2f}) selfMask={}".format(
                area, error, self.threshold_mode, self.roi_top_ratio, self.roi_bottom_ratio, self.self_mask_filter_enable)
            cv2.putText(debug, "LINE LOST", (18, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3, cv2.LINE_AA)

        cv2.putText(debug, "ROI black line -> center error -> PID", (18, h - 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(debug, status, (18, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                    (0, 255, 0) if online else (0, 0, 255), 2, cv2.LINE_AA)

        self.error_pub.publish(Float32(data=float(error)))
        self.online_pub.publish(Bool(data=bool(online)))
        self.center_x_pub.publish(Float32(data=float(x1 + cx_roi) if online and cx_roi is not None else -1.0))
        self.status_pub.publish(String(data=status))

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)
            if self.publish_mask:
                mask_full = np.zeros((h, w), dtype=np.uint8)
                mask_full[y1:y2, x1:x2] = mask
                mask_msg = self.bridge.cv2_to_imgmsg(mask_full, encoding="mono8")
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge publish failed: %s", exc)


def main():
    CameraLineDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
