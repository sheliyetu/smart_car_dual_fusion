#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID controller for camera-based line following with lost-line recovery.

Input error convention:
  /line_follower/error > 0 means the black line center appears to image right.
ROS angular convention:
  +angular.z turns left, -angular.z turns right. So turn_sign defaults to -1.

Lost-line recovery is intentionally NOT a pure spin.  A pure spin cannot recover
when the robot has drifted far from the line and the line is outside the camera
field of view.  The recovery state machine uses: slow arc search -> short reverse
arc -> alternating expanding arc search, so the camera sweeps a larger ground area.
"""
from __future__ import annotations

import math

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String


def param(name, default):
    return rospy.get_param("~" + name, rospy.get_param("/" + name, default))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class CameraLineFollowerNode(object):
    def __init__(self):
        rospy.init_node("camera_line_follower_node")

        self.error = 0.0
        self.online = False
        self.last_online_time = rospy.Time.now()
        self.lost_start_time = None
        self.recovery_dir = 1.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.last_safe_action_time = rospy.Time(0)

        self.integral = 0.0
        self.last_error = 0.0
        self.last_d = 0.0
        self.last_param_refresh = rospy.Time(0)
        self.prev_control_mode = None

        self.load_params(force=True)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.steer_pub = rospy.Publisher("/line_follower/steer", Float32, queue_size=5)
        self.pid_pub = rospy.Publisher("/line_follower/pid_terms", Float32MultiArray, queue_size=5)
        self.wheel_pwm_pub = rospy.Publisher("/line_follower/wheel_pwm", Float32MultiArray, queue_size=5)
        self.state_pub = rospy.Publisher("/line_follower/state", String, queue_size=5)

        rospy.Subscriber("/line_follower/error", Float32, self.error_cb, queue_size=1)
        rospy.Subscriber("/line_follower/online", Bool, self.online_cb, queue_size=1)
        rospy.Subscriber("/line_follower/reset_pid", Empty, self.reset_pid_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self.timer_cb)
        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("camera_line_follower_node started: mode=%s kp=%.2f ki=%.2f kd=%.2f recovery=%s",
                      self.control_mode, self.kp, self.ki, self.kd, self.lost_strategy)

    def load_params(self, force=False):
        now = rospy.Time.now()
        if not force and (now - self.last_param_refresh).to_sec() < 0.25:
            return
        self.last_param_refresh = now

        self.enabled = bool(param("enabled", True))

        self.control_mode = str(param("control_mode", "pid")).lower()
        if self.control_mode not in ("p", "pd", "pi", "pid"):
            self.control_mode = "pid"
        if self.prev_control_mode is not None and self.prev_control_mode != self.control_mode:
            self.reset_pid()
        self.prev_control_mode = self.control_mode

        self.kp = float(param("kp", 38.0))
        self.ki = float(param("ki", 0.0))
        self.kd = float(param("kd", 5.0))
        self.pos_out_limit = float(param("pos_out_limit", 60.0))
        self.pos_int_limit = float(param("pos_int_limit", 2.0))
        self.loop_rate = float(param("loop_rate", 50.0))

        self.base_speed_pct = float(param("base_speed_pct", 32.0))
        self.base_min_pct = float(param("base_min_pct", 16.0))
        self.curve_slow = bool(param("curve_slow", True))
        self.curve_factor = float(param("curve_factor", 0.55))
        self.max_linear = float(param("max_linear", 0.55))
        self.max_angular = float(param("max_angular", 2.6))
        self.turn_sign = float(param("turn_sign", -1.0))

        # Backward compatible switches.  For robust recovery, keep stop_when_lost False.
        self.stop_when_lost = bool(param("stop_when_lost", False))
        self.lost_stop_after = float(param("lost_stop_after", 4.0))
        self.keep_turn_when_lost = bool(param("keep_turn_when_lost", True))

        # New recovery parameters.
        self.lost_strategy = str(param("lost_strategy", "arc_recovery")).lower()  # arc_recovery | spin | stop
        self.lost_grace_time = float(param("lost_grace_time", 0.20))
        self.lost_reverse_after = float(param("lost_reverse_after", 0.75))
        self.lost_reverse_duration = float(param("lost_reverse_duration", 0.55))
        self.lost_scan_period = float(param("lost_scan_period", 1.20))
        self.lost_forward_pct = float(param("lost_forward_pct", 11.0))
        self.lost_reverse_pct = float(param("lost_reverse_pct", -8.0))
        self.lost_turn_pct = float(param("lost_turn_pct", 32.0))
        self.lost_spin_turn_pct = float(param("lost_spin_turn_pct", 26.0))

        # Safety guard for classroom demos.  A camera line follower can lose the
        # line completely; if it keeps moving, Gazebo's empty world lets it drive
        # away forever.  These parameters stop or reset the robot before that.
        self.lost_safe_stop_after = float(param("lost_safe_stop_after", 2.0))
        self.lost_auto_reset_after = float(param("lost_auto_reset_after", 4.5))
        self.out_of_bounds_enable = bool(param("out_of_bounds_enable", True))
        self.out_of_bounds_action = str(param("out_of_bounds_action", "reset")).lower()  # stop | reset
        self.x_abs_limit = float(param("x_abs_limit", 4.0))
        self.y_abs_limit = float(param("y_abs_limit", 3.0))
        self.reset_model_name = str(param("reset_model_name", param("robot_model_name", "three_wheel_camera_car")))
        self.reset_x = float(param("reset_x", 2.136986))
        self.reset_y = float(param("reset_y", -0.207531))
        self.reset_z = float(param("reset_z", 0.04))
        self.reset_yaw = float(param("reset_yaw", 1.232498))
        self.reset_cooldown = float(param("reset_cooldown", 2.0))

    def reset_pid(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_d = 0.0

    def reset_recovery(self):
        self.lost_start_time = None
        self.recovery_dir = 1.0 if self.last_error >= 0.0 else -1.0

    def reset_pid_cb(self, _msg):
        self.reset_pid()
        self.reset_recovery()
        rospy.loginfo("PID state reset by tuner GUI")

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def error_cb(self, msg):
        self.error = float(clamp(msg.data, -1.0, 1.0))

    def online_cb(self, msg):
        was_online = self.online
        self.online = bool(msg.data)
        if self.online:
            self.last_online_time = rospy.Time.now()
            self.reset_recovery()
            if not was_online:
                self.reset_pid()

    def pid_compute(self, error, dt):
        if abs(error) < 0.005:
            error = 0.0
        p = self.kp * error

        if abs(error) < 0.75:
            self.integral += error * dt
        else:
            self.integral *= 0.90
        self.integral = clamp(self.integral, -self.pos_int_limit, self.pos_int_limit)
        i = self.ki * self.integral

        d_raw = (error - self.last_error) / dt if dt > 1e-6 else 0.0
        self.last_d = 0.45 * d_raw + 0.55 * self.last_d
        d = self.kd * self.last_d

        ep, ei, ed = p, i, d
        if self.control_mode == "p":
            ei, ed = 0.0, 0.0
        elif self.control_mode == "pd":
            ei = 0.0
        elif self.control_mode == "pi":
            ed = 0.0

        out = clamp(ep + ei + ed, -self.pos_out_limit, self.pos_out_limit)
        self.last_error = error
        return out, ep, ei, ed

    def twist_from_pct(self, linear_pct, steer_pct):
        msg = Twist()
        msg.linear.x = (linear_pct / 100.0) * self.max_linear
        msg.angular.z = self.turn_sign * (steer_pct / 100.0) * self.max_angular
        left_pct = clamp(linear_pct + steer_pct, -100.0, 100.0)
        right_pct = clamp(linear_pct - steer_pct, -100.0, 100.0)
        return msg, left_pct, right_pct, linear_pct

    def make_twist(self, steer_pct, force_stop=False):
        if force_stop:
            return Twist(), 0.0, 0.0, 0.0

        base = self.base_speed_pct
        if self.curve_slow:
            min_speed = min(self.base_min_pct, self.base_speed_pct)
            base = clamp(self.base_speed_pct - abs(steer_pct) * self.curve_factor, min_speed, self.base_speed_pct)

        return self.twist_from_pct(base, steer_pct)

    def quaternion_from_yaw(self, yaw):
        qz = math.sin(0.5 * yaw)
        qw = math.cos(0.5 * yaw)
        return qz, qw

    def reset_robot_to_start(self, reason="safety"):
        now = rospy.Time.now()
        if (now - self.last_safe_action_time).to_sec() < self.reset_cooldown:
            return False
        self.last_safe_action_time = now
        self.stop_robot()
        try:
            qz, qw = self.quaternion_from_yaw(self.reset_yaw)
            state = ModelState()
            state.model_name = self.reset_model_name
            state.pose.position.x = self.reset_x
            state.pose.position.y = self.reset_y
            state.pose.position.z = self.reset_z
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            state.twist = Twist()
            state.reference_frame = "world"
            self.set_model_state(state)
            self.reset_pid()
            self.reset_recovery()
            rospy.logwarn("Reset %s to start pose because %s", self.reset_model_name, reason)
            return True
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Failed to reset model state: %s", exc)
            return False

    def out_of_bounds(self):
        return self.out_of_bounds_enable and (abs(self.odom_x) > self.x_abs_limit or abs(self.odom_y) > self.y_abs_limit)

    def lost_recovery_cmd(self, lost_duration):
        """Return twist + diagnostic values during line loss.

        Phase A, grace: crawl forward and turn toward the last observed side.
        Phase B, reverse arc: if the robot has overshot the line, back up while sweeping.
        Phase C, alternating arc scan: do not spin in place; move slowly while sweeping left/right.
        """
        if self.lost_strategy == "stop" or (self.stop_when_lost and lost_duration > self.lost_stop_after):
            return Twist(), 0.0, 0.0, 0.0, "lost_stop"

        # Hard safety stop: after a short recovery window, do not keep driving
        # blind.  The auto-reset branch in timer_cb can move the robot back to
        # the start pose after a longer timeout.
        if self.lost_safe_stop_after > 0.0 and lost_duration > self.lost_safe_stop_after:
            return Twist(), 0.0, 0.0, 0.0, "lost_safe_stop"

        direction = self.recovery_dir
        if self.lost_strategy == "spin":
            steer = self.lost_spin_turn_pct * direction
            return self.twist_from_pct(0.0, steer) + ("lost_spin",)

        if lost_duration < self.lost_grace_time:
            steer = self.lost_turn_pct * direction
            linear = self.lost_forward_pct
            return self.twist_from_pct(linear, steer) + ("lost_arc_grace",)

        if lost_duration < self.lost_reverse_after + self.lost_reverse_duration:
            # Reverse while sweeping opposite to the last visible side to undo overshoot.
            steer = -0.65 * self.lost_turn_pct * direction
            linear = self.lost_reverse_pct
            return self.twist_from_pct(linear, steer) + ("lost_reverse_arc",)

        t = max(0.0, lost_duration - self.lost_reverse_after - self.lost_reverse_duration)
        phase = int(t / max(0.2, self.lost_scan_period))
        alt = direction if phase % 2 == 0 else -direction
        # Slowly widen the sweep but keep a little translation so the camera covers new ground.
        gain = min(1.45, 1.0 + 0.12 * phase)
        steer = clamp(self.lost_turn_pct * gain * alt, -55.0, 55.0)
        linear = max(5.0, self.lost_forward_pct * max(0.55, 1.0 - 0.06 * phase))
        return self.twist_from_pct(linear, steer) + ("lost_alternating_arc",)

    def timer_cb(self, _event):
        self.load_params(force=False)
        dt = 1.0 / max(1.0, self.loop_rate)
        now = rospy.Time.now()

        if not self.enabled:
            self.cmd_pub.publish(Twist())
            self.steer_pub.publish(Float32(data=0.0))
            self.pid_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            self.wheel_pwm_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            self.state_pub.publish(String(data="paused_by_gui enabled=false"))
            return

        if self.out_of_bounds():
            state = "out_of_bounds x={:+.2f} y={:+.2f} action={}".format(
                self.odom_x, self.odom_y, self.out_of_bounds_action)
            if self.out_of_bounds_action == "reset":
                self.reset_robot_to_start("out_of_bounds")
            self.cmd_pub.publish(Twist())
            self.steer_pub.publish(Float32(data=0.0))
            self.pid_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            self.wheel_pwm_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            self.state_pub.publish(String(data=state))
            return

        if self.online:
            steer, p, i, d = self.pid_compute(self.error, dt)
            twist, left_pct, right_pct, base = self.make_twist(steer, force_stop=False)
            state = "online error={:+.3f} steer={:+.1f} base={:.1f}".format(self.error, steer, base)
        else:
            if self.lost_start_time is None:
                self.lost_start_time = now
                self.recovery_dir = 1.0 if self.last_error >= 0.0 else -1.0
                self.reset_pid()
            lost_duration = (now - self.lost_start_time).to_sec()
            if self.lost_auto_reset_after > 0.0 and lost_duration > self.lost_auto_reset_after:
                self.reset_robot_to_start("lost_line_timeout")
                twist, left_pct, right_pct, base = Twist(), 0.0, 0.0, 0.0
                steer, p, i, d = 0.0, 0.0, 0.0, 0.0
                state = "lost_auto_reset t={:.2f}s".format(lost_duration)
            else:
                twist, left_pct, right_pct, base, phase = self.lost_recovery_cmd(lost_duration)
                steer = 0.5 * (left_pct - right_pct)
                p, i, d = steer, 0.0, 0.0
                state = "{} t={:.2f}s dir={:+.0f} last_err={:+.3f} steer={:+.1f} lin={:+.1f}".format(
                    phase, lost_duration, self.recovery_dir, self.last_error, steer, base)

        self.cmd_pub.publish(twist)
        self.steer_pub.publish(Float32(data=float(steer)))
        self.pid_pub.publish(Float32MultiArray(data=[float(p), float(i), float(d)]))
        self.wheel_pwm_pub.publish(Float32MultiArray(data=[float(left_pct), float(right_pct), float(base)]))
        self.state_pub.publish(String(data=state))

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main():
    CameraLineFollowerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
