#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz markers for the camera line follower.

Publishes:
  /line_follower/track_markers   black line and start marker in odom frame
  /line_follower/camera_markers  moving error arrow/status in base_link frame
"""
from __future__ import annotations

import os
import sys

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from track_utils import generate_track  # noqa: E402


def param(name, default):
    return rospy.get_param("~" + name, rospy.get_param("/" + name, default))


class LineVisualizerNode(object):
    def __init__(self):
        rospy.init_node("line_visualizer_node")
        self.line_width = float(param("line_width", 0.10))
        self.marker_rate = float(param("marker_rate", 20.0))
        self.error = 0.0
        self.steer = 0.0
        self.online = False
        self.state = "waiting image"
        self.track = generate_track(samples_per_seg=12)

        self.track_pub = rospy.Publisher("/line_follower/track_markers", MarkerArray, queue_size=1, latch=True)
        self.camera_pub = rospy.Publisher("/line_follower/camera_markers", MarkerArray, queue_size=2)

        rospy.Subscriber("/line_follower/error", Float32, self.error_cb, queue_size=1)
        rospy.Subscriber("/line_follower/steer", Float32, self.steer_cb, queue_size=1)
        rospy.Subscriber("/line_follower/online", Bool, self.online_cb, queue_size=1)
        rospy.Subscriber("/line_follower/state", String, self.state_cb, queue_size=1)

        self.publish_track_markers()
        rospy.Timer(rospy.Duration(1.0 / self.marker_rate), self.publish_camera_markers)
        rospy.loginfo("line_visualizer_node started")

    @staticmethod
    def color(marker, r, g, b, a=1.0):
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

    def error_cb(self, msg):
        self.error = float(msg.data)

    def steer_cb(self, msg):
        self.steer = float(msg.data)

    def online_cb(self, msg):
        self.online = bool(msg.data)

    def state_cb(self, msg):
        self.state = msg.data

    def publish_track_markers(self):
        arr = MarkerArray()
        now = rospy.Time.now()
        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = now
        line.ns = "track"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = self.line_width
        self.color(line, 0.0, 0.0, 0.0, 1.0)
        for x, y in self.track:
            line.points.append(Point(x=x, y=y, z=0.035))
        x0, y0 = self.track[0]
        line.points.append(Point(x=x0, y=y0, z=0.035))
        arr.markers.append(line)

        start = Marker()
        start.header.frame_id = "odom"
        start.header.stamp = now
        start.ns = "track"
        start.id = 1
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x = x0
        start.pose.position.y = y0
        start.pose.position.z = 0.065
        start.pose.orientation.w = 1.0
        start.scale.x = 0.12
        start.scale.y = 0.12
        start.scale.z = 0.04
        self.color(start, 0.1, 0.85, 0.2, 1.0)
        arr.markers.append(start)

        label = Marker()
        label.header.frame_id = "odom"
        label.header.stamp = now
        label.ns = "track_text"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x0
        label.pose.position.y = y0
        label.pose.position.z = 0.22
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.text = "START"
        self.color(label, 0.1, 0.9, 0.2, 1.0)
        arr.markers.append(label)
        self.track_pub.publish(arr)

    def publish_camera_markers(self, _event):
        arr = MarkerArray()
        now = rospy.Time.now()

        # ROI / lookahead bar in the robot base frame.
        bar = Marker()
        bar.header.frame_id = "base_link"
        bar.header.stamp = now
        bar.ns = "camera_control"
        bar.id = 0
        bar.type = Marker.CUBE
        bar.action = Marker.ADD
        bar.pose.position.x = 0.34
        bar.pose.position.y = 0.0
        bar.pose.position.z = 0.115
        bar.pose.orientation.w = 1.0
        bar.scale.x = 0.015
        bar.scale.y = 0.36
        bar.scale.z = 0.015
        self.color(bar, 0.1, 0.45, 1.0, 0.55)
        arr.markers.append(bar)

        # Error arrow: positive image error is to robot right, i.e. negative ROS y.
        arrow = Marker()
        arrow.header.frame_id = "base_link"
        arrow.header.stamp = now
        arrow.ns = "camera_control"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.points.append(Point(x=0.34, y=0.0, z=0.15))
        arrow.points.append(Point(x=0.34, y=-0.22 * self.error, z=0.15))
        arrow.scale.x = 0.018
        arrow.scale.y = 0.045
        arrow.scale.z = 0.045
        if self.online:
            self.color(arrow, 1.0, 0.55, 0.0, 1.0)
        else:
            self.color(arrow, 1.0, 0.0, 0.0, 1.0)
        arr.markers.append(arrow)

        status = Marker()
        status.header.frame_id = "base_link"
        status.header.stamp = now
        status.ns = "camera_control_text"
        status.id = 2
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = 0.02
        status.pose.position.y = 0.0
        status.pose.position.z = 0.32
        status.pose.orientation.w = 1.0
        status.scale.z = 0.055
        status.text = "camera line\nerr {:+.2f} | steer {:+.1f}\n{}".format(self.error, self.steer, self.state[:40])
        if self.online:
            self.color(status, 0.3, 1.0, 0.35, 1.0)
        else:
            self.color(status, 1.0, 0.15, 0.1, 1.0)
        arr.markers.append(status)

        # Simple camera frustum hint.
        frustum = Marker()
        frustum.header.frame_id = "camera_link"
        frustum.header.stamp = now
        frustum.ns = "camera_frustum"
        frustum.id = 3
        frustum.type = Marker.LINE_LIST
        frustum.action = Marker.ADD
        frustum.pose.orientation.w = 1.0
        frustum.scale.x = 0.01
        self.color(frustum, 0.2, 0.9, 1.0, 0.65)
        origin = Point(x=0.0, y=0.0, z=0.0)
        pts = [Point(x=0.45, y=0.22, z=0.16), Point(x=0.45, y=-0.22, z=0.16),
               Point(x=0.45, y=0.22, z=-0.16), Point(x=0.45, y=-0.22, z=-0.16)]
        for p in pts:
            frustum.points.append(origin)
            frustum.points.append(p)
        edges = [(0, 1), (1, 3), (3, 2), (2, 0)]
        for a, b in edges:
            frustum.points.append(pts[a])
            frustum.points.append(pts[b])
        arr.markers.append(frustum)

        self.camera_pub.publish(arr)


def main():
    LineVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
