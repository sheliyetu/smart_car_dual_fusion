#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Virtual 5-channel photoelectric sensor node for Gazebo.

This node intentionally simulates the same sensing chain as the HTML demo:
white ground -> high ADC, black line -> low ADC, 5 sensors placed in front of the car.
It does not require a custom Gazebo plugin, so it is easy to run in ROS1 Noetic.
"""
from __future__ import annotations

import math
import os
import random
import sys
from typing import List, Tuple

import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from track_utils import generate_track, min_distance_to_track  # noqa: E402


def param(name, default):
    """Read private param first, then global param, then default."""
    return rospy.get_param("~" + name, rospy.get_param("/" + name, default))


class LineSensorNode(object):
    def __init__(self):
        rospy.init_node("line_sensor_node")

        self.robot_model_name = param("robot_model_name", "three_wheel_camera_car")
        self.sensor_rate = float(param("sensor_rate", 100.0))
        self.marker_rate = float(param("marker_rate", 20.0))
        self.last_param_refresh = rospy.Time(0)
        self.line_width_last = None
        self.load_runtime_params(force=True)
        self.threshold = float(param("fixed_threshold", 2000.0))

        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.raw = [self.white_adc] * len(self.sensor_y_offsets)
        self.binary = [0] * len(self.sensor_y_offsets)
        self.error = 0.0
        self.online = True
        self.track_type = param("track_type", "default")
        self.track_reverse = bool(param("track_reverse", False))
        self.track = generate_track(track_type=self.track_type, samples_per_seg=12,
                                    reverse=self.track_reverse)

        self.raw_pub = rospy.Publisher("/line_follower/raw_adc", Float32MultiArray, queue_size=5)
        self.sensor_marker_pub = rospy.Publisher("/line_follower/sensor_markers", MarkerArray, queue_size=2)
        self.track_marker_pub = rospy.Publisher("/line_follower/track_markers", MarkerArray, queue_size=1, latch=True)
        self.sensor_error_pub = rospy.Publisher("/line_follower/sensor_error", Float32, queue_size=5)
        self.sensor_online_pub = rospy.Publisher("/line_follower/sensor_online", Bool, queue_size=5)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber("/line_follower/threshold", Float32, self.threshold_cb, queue_size=1)
        rospy.Subscriber("/line_follower/binary", Int32MultiArray, self.binary_cb, queue_size=1)
        rospy.Subscriber("/line_follower/sensor_error", Float32, self.error_cb, queue_size=1)
        rospy.Subscriber("/line_follower/sensor_online", Bool, self.online_cb, queue_size=1)

        self.publish_track_markers()
        rospy.Timer(rospy.Duration(1.0 / self.sensor_rate), self.sensor_timer_cb)
        rospy.Timer(rospy.Duration(1.0 / self.marker_rate), self.marker_timer_cb)

        rospy.loginfo("line_sensor_node started: 5-channel ADC sensor, robot_model=%s", self.robot_model_name)


    def load_runtime_params(self, force=False):
        """Refresh sensor-simulation parameters so the GUI sliders take effect live."""
        now = rospy.Time.now()
        if not force and (now - self.last_param_refresh).to_sec() < 0.25:
            return
        self.last_param_refresh = now
        self.sensor_forward = float(param("sensor_forward", 0.18))
        self.sensor_y_offsets = [float(v) for v in param("sensor_y_offsets", [0.08, 0.04, 0.0, -0.04, -0.08])]
        self.sensor_spacing = abs(self.sensor_y_offsets[0] - self.sensor_y_offsets[1]) if len(self.sensor_y_offsets) > 1 else 0.04
        self.sensor_pins = [int(v) for v in param("sensor_pins", [27, 33, 32, 35, 34])]
        old_line_width = getattr(self, "line_width", None)
        self.line_width = float(param("line_width", 0.10))
        self.noise_adc = float(param("noise_adc", 60.0))
        self.white_adc = float(param("white_adc", 3500.0))
        self.black_adc = float(param("black_adc", 800.0))
        if old_line_width is not None and abs(old_line_width - self.line_width) > 1e-6:
            try:
                self.publish_track_markers()
            except Exception:
                pass

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        self.x = pose.position.x
        self.y = pose.position.y
        q = pose.orientation
        _, _, self.yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_ready = True

    def threshold_cb(self, msg):
        self.threshold = msg.data

    def binary_cb(self, msg):
        self.binary = list(msg.data)

    def error_cb(self, msg):
        self.error = msg.data

    def online_cb(self, msg):
        self.online = msg.data

    def sensor_world_positions(self) -> List[Tuple[float, float]]:
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        out = []
        for sy in self.sensor_y_offsets:
            sx = self.sensor_forward
            wx = self.x + c * sx - s * sy
            wy = self.y + s * sx + c * sy
            out.append((wx, wy))
        return out

    def sample_adc(self, wx: float, wy: float) -> float:
        d = min_distance_to_track(wx, wy, self.track)
        half = 0.5 * self.line_width
        edge = 0.025
        if d <= half:
            blackness = 1.0
        elif d <= half + edge:
            blackness = 1.0 - (d - half) / edge
        else:
            blackness = 0.0
        adc = self.white_adc - (self.white_adc - self.black_adc) * blackness
        adc += (random.random() - 0.5) * self.noise_adc
        return max(0.0, min(4095.0, adc))

    def sensor_timer_cb(self, _event):
        self.load_runtime_params(force=False)
        if not self.pose_ready:
            return
        positions = self.sensor_world_positions()
        self.raw = [self.sample_adc(x, y) for x, y in positions]
        msg = Float32MultiArray()
        msg.data = [float(round(v, 1)) for v in self.raw]
        self.raw_pub.publish(msg)

        # Compute and publish sensor error (binary centroid)
        s_weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
        binary = [1 if v < self.threshold else 0 for v in self.raw]
        s = sum(binary)
        contrast = max(self.raw) - min(self.raw)
        online = contrast >= 300 and s > 0
        if s > 0:
            error = sum(s_weights[i] * binary[i] for i in range(min(5, len(binary)))) / s
        else:
            error = 0.0 if not online else self.error
        self.error = error
        self.online = online
        self.sensor_error_pub.publish(Float32(data=error))
        self.sensor_online_pub.publish(Bool(data=online))

    @staticmethod
    def color_marker(marker, r, g, b, a=1.0):
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

    def publish_track_markers(self):
        arr = MarkerArray()
        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = rospy.Time.now()
        line.ns = "track"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = self.line_width
        self.color_marker(line, 0.0, 0.0, 0.0, 1.0)
        for x, y in self.track:
            line.points.append(Point(x=x, y=y, z=0.025))
        # close curve
        x0, y0 = self.track[0]
        line.points.append(Point(x=x0, y=y0, z=0.025))
        arr.markers.append(line)

        start = Marker()
        start.header.frame_id = "odom"
        start.header.stamp = rospy.Time.now()
        start.ns = "track"
        start.id = 1
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x = x0
        start.pose.position.y = y0
        start.pose.position.z = 0.055
        start.pose.orientation.w = 1.0
        start.scale.x = 0.12
        start.scale.y = 0.12
        start.scale.z = 0.04
        self.color_marker(start, 0.1, 0.8, 0.2, 1.0)
        arr.markers.append(start)

        self.track_marker_pub.publish(arr)

    def marker_timer_cb(self, _event):
        self.load_runtime_params(force=False)
        if not self.pose_ready:
            return
        positions = self.sensor_world_positions()
        arr = MarkerArray()
        now = rospy.Time.now()
        labels = [u"左2", u"左1", u"中心", u"右1", u"右2"]

        # Individual sensor spheres and ADC text.
        for i, (wx, wy) in enumerate(positions):
            is_black = False
            if i < len(self.binary):
                is_black = self.binary[i] == 1
            else:
                is_black = self.raw[i] < self.threshold

            sphere = Marker()
            sphere.header.frame_id = "odom"
            sphere.header.stamp = now
            sphere.ns = "sensors"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wx
            sphere.pose.position.y = wy
            sphere.pose.position.z = 0.065
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.055
            sphere.scale.y = 0.055
            sphere.scale.z = 0.025
            if is_black:
                self.color_marker(sphere, 1.0, 0.25, 0.05, 1.0)
            else:
                self.color_marker(sphere, 0.3, 0.6, 1.0, 0.75)
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = "odom"
            text.header.stamp = now
            text.ns = "sensor_text"
            text.id = i + 20
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wx
            text.pose.position.y = wy
            text.pose.position.z = 0.16
            text.pose.orientation.w = 1.0
            text.scale.z = 0.055
            pin = self.sensor_pins[i] if i < len(self.sensor_pins) else i
            text.text = "GPIO{} {}\nADC {:.0f}".format(pin, labels[i], self.raw[i])
            self.color_marker(text, 0.95, 0.95, 0.95, 1.0)
            arr.markers.append(text)

        # Error arrow: from sensor center to estimated line lateral offset.
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        cx = self.x + c * self.sensor_forward
        cy = self.y + s * self.sensor_forward
        lateral_y = -self.error * self.sensor_spacing
        ex = cx - s * lateral_y
        ey = cy + c * lateral_y

        arrow = Marker()
        arrow.header.frame_id = "odom"
        arrow.header.stamp = now
        arrow.ns = "line_error"
        arrow.id = 100
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.points.append(Point(x=cx, y=cy, z=0.11))
        arrow.points.append(Point(x=ex, y=ey, z=0.11))
        arrow.scale.x = 0.018
        arrow.scale.y = 0.045
        arrow.scale.z = 0.045
        if self.online:
            self.color_marker(arrow, 1.0, 0.6, 0.0, 1.0)
        else:
            self.color_marker(arrow, 1.0, 0.0, 0.0, 1.0)
        arr.markers.append(arrow)

        status = Marker()
        status.header.frame_id = "odom"
        status.header.stamp = now
        status.ns = "line_status"
        status.id = 101
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = self.x
        status.pose.position.y = self.y
        status.pose.position.z = 0.36
        status.pose.orientation.w = 1.0
        status.scale.z = 0.08
        status.text = "error={:+.2f}  {}".format(self.error, "ONLINE" if self.online else "LOST")
        if self.online:
            self.color_marker(status, 0.3, 1.0, 0.3, 1.0)
        else:
            self.color_marker(status, 1.0, 0.1, 0.1, 1.0)
        arr.markers.append(status)

        self.sensor_marker_pub.publish(arr)


if __name__ == "__main__":
    try:
        LineSensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
