#!/usr/bin/env python3
"""Kalman filter fusion for camera + photoelectric sensor.

State: x = [error, error_rate]^T
- error: line deviation in [-2, 2]
- error_rate: change rate of error (derivative)

Process model (constant velocity):
  x_k = F @ x_{k-1} + w,  w ~ N(0, Q)

Measurement model:
  z_camera = H @ x + v_c,  v_c ~ N(0, R_camera)
  z_sensor = H @ x + v_s,  v_s ~ N(0, R_sensor)

Key: R values are ADAPTIVE — based on sensor confidence signals.
"""
import math
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool


class KalmanFusion:
    def __init__(self):
        rospy.init_node("sensor_fusion")

        # State: [error, error_rate]
        self.x = np.zeros(2, dtype=np.float64)
        self.P = np.eye(2, dtype=np.float64) * 0.1

        dt = 1.0 / float(rospy.get_param("~loop_rate", 50.0))

        # Process model
        self.F = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)

        # Process noise
        process_noise = float(rospy.get_param("~process_noise", 0.01))
        self.Q = np.array([[process_noise * dt, 0.0],
                           [0.0, process_noise * 2.0]], dtype=np.float64)

        # Observation model: measure error directly
        self.H = np.array([[1.0, 0.0]], dtype=np.float64)

        # Base observation noise
        self.R_base = float(rospy.get_param("~R_base", 0.1))

        # Sensor inputs
        self.camera_error = 0.0
        self.camera_online = False
        self.sensor_error = 0.0
        self.sensor_online = False

        # Rolling variance estimation (window = 10)
        self.window_size = 10
        self.camera_errors = []
        self.sensor_errors = []

        self.loop_rate = float(rospy.get_param("~loop_rate", 50.0))

        rospy.Subscriber("/line_follower/camera_error", Float32, self.camera_cb)
        rospy.Subscriber("/line_follower/camera_online", Bool, self.camera_online_cb)
        rospy.Subscriber("/line_follower/sensor_error", Float32, self.sensor_cb)
        rospy.Subscriber("/line_follower/sensor_online", Bool, self.sensor_online_cb)

        self.fused_pub = rospy.Publisher("/line_follower/error", Float32, queue_size=5)
        self.online_pub = rospy.Publisher("/line_follower/online", Bool, queue_size=5)

        rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self.timer_cb)
        rospy.loginfo("Kalman fusion started: R_base=%.4f, process_noise=%.4f",
                      self.R_base, process_noise)

    def camera_cb(self, msg): self.camera_error = msg.data
    def camera_online_cb(self, msg): self.camera_online = msg.data
    def sensor_cb(self, msg): self.sensor_error = msg.data
    def sensor_online_cb(self, msg): self.sensor_online = msg.data

    def _rolling_var(self, buf, val, window):
        buf.append(val)
        if len(buf) > window:
            buf.pop(0)
        if len(buf) < 3:
            return 0.01
        return float(np.var(buf)) + 1e-6

    def _adaptive_R(self, base_R, rolling_var, online, fallback_var=10.0):
        """Adaptive observation noise: higher variance → higher R → lower trust."""
        if not online:
            return fallback_var
        return base_R * (1.0 + rolling_var / (base_R + 1e-6))

    def _predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Clamp state to prevent divergence
        self.x[0] = max(-3.0, min(3.0, self.x[0]))
        self.x[1] = max(-5.0, min(5.0, self.x[1]))

    def _update(self, z, R):
        """Kalman update step with scalar measurement."""
        y = z - (self.H @ self.x)[0]      # innovation
        S = (self.H @ self.P @ self.H.T)[0, 0] + R  # innovation covariance
        K = (self.P @ self.H.T)[:, 0] / S  # Kalman gain
        self.x = self.x + K * y
        self.P = (np.eye(2) - np.outer(K, self.H[0])) @ self.P
        self.x[0] = max(-2.5, min(2.5, self.x[0]))

    def timer_cb(self, _event):
        self._predict()

        # Rolling variance
        cam_var = self._rolling_var(self.camera_errors, self.camera_error, self.window_size)
        sen_var = self._rolling_var(self.sensor_errors, self.sensor_error, self.window_size)

        # Adaptive R
        R_cam = self._adaptive_R(self.R_base, cam_var, self.camera_online)
        R_sen = self._adaptive_R(self.R_base, sen_var, self.sensor_online)

        # Update in order: camera first (lower R → higher weight)
        if self.camera_online:
            self._update(self.camera_error, R_cam)
        if self.sensor_online:
            self._update(self.sensor_error, R_sen)

        # If both offline, predict only (coast with motion model)
        online = self.camera_online or self.sensor_online
        if not online:
            self.x[0] *= 0.95  # slow decay toward 0

        fused = max(-2.0, min(2.0, self.x[0]))
        self.fused_pub.publish(Float32(data=fused))
        self.online_pub.publish(Bool(data=online))


if __name__ == "__main__":
    try:
        KalmanFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
