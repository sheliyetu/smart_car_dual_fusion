#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small terminal dashboard for the camera line follower."""
from __future__ import annotations

import sys

import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


class CameraDashboardNode(object):
    def __init__(self):
        rospy.init_node("camera_dashboard_node")
        self.error = 0.0
        self.steer = 0.0
        self.online = False
        self.pid = [0.0, 0.0, 0.0]
        self.wheels = [0.0, 0.0, 0.0]
        self.vision_state = "waiting image"
        self.state = "waiting control"

        rospy.Subscriber("/line_follower/error", Float32, lambda m: setattr(self, "error", m.data), queue_size=1)
        rospy.Subscriber("/line_follower/steer", Float32, lambda m: setattr(self, "steer", m.data), queue_size=1)
        rospy.Subscriber("/line_follower/online", Bool, lambda m: setattr(self, "online", m.data), queue_size=1)
        rospy.Subscriber("/line_follower/pid_terms", Float32MultiArray, self.pid_cb, queue_size=1)
        rospy.Subscriber("/line_follower/wheel_pwm", Float32MultiArray, self.wheel_cb, queue_size=1)
        rospy.Subscriber("/line_follower/vision_state", String, lambda m: setattr(self, "vision_state", m.data), queue_size=1)
        rospy.Subscriber("/line_follower/state", String, lambda m: setattr(self, "state", m.data), queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.timer_cb)

    def pid_cb(self, msg):
        data = list(msg.data)
        if len(data) >= 3:
            self.pid = data[:3]

    def wheel_cb(self, msg):
        data = list(msg.data)
        if len(data) >= 3:
            self.wheels = data[:3]

    def timer_cb(self, _event):
        line = (
            "\rCAM {status:<6} err={err:+.3f} steer={steer:+6.1f} | "
            "P={p:+6.2f} I={i:+6.2f} D={d:+6.2f} | "
            "L={l:+5.1f}% R={r:+5.1f}% base={b:4.1f}% | {state:<45}"
        ).format(
            status="ONLINE" if self.online else "LOST",
            err=self.error,
            steer=self.steer,
            p=self.pid[0], i=self.pid[1], d=self.pid[2],
            l=self.wheels[0], r=self.wheels[1], b=self.wheels[2],
            state=self.state[:45],
        )
        sys.stdout.write(line)
        sys.stdout.flush()


def main():
    CameraDashboardNode()
    rospy.spin()


if __name__ == "__main__":
    main()
