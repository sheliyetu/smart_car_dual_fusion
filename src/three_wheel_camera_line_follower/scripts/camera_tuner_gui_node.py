#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTML-like visual parameter tuner for the camera line follower.

This node is intentionally lightweight: it writes ROS parameters directly, while
camera_line_detector_node and camera_line_follower_node refresh those parameters
at runtime.  It provides sliders, mode buttons, presets, live error/PID/wheel
status and Gazebo reset/start-stop controls.
"""
from __future__ import annotations

import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String
from std_srvs.srv import Empty


PARAM_DEFAULTS = {
    # control
    "enabled": True,
    "control_mode": "pid",
    "kp": 38.0,
    "ki": 0.0,
    "kd": 5.0,
    "base_speed_pct": 32.0,
    "base_min_pct": 16.0,
    "curve_factor": 0.55,
    "pos_out_limit": 60.0,
    "max_linear": 0.55,
    "max_angular": 2.6,
    # vision
    "threshold_mode": "otsu",
    "roi_top_ratio": 0.55,
    "roi_bottom_ratio": 0.95,
    "roi_left_ratio": 0.05,
    "roi_right_ratio": 0.95,
    "black_threshold": 90,
    "min_area": 180.0,
    "min_black_ratio": 0.002,
    "error_filter_alpha": 0.60,
    "blur_kernel": 5,
    "morph_kernel": 5,
    # recovery
    "lost_strategy": "arc_recovery",
    "lost_grace_time": 0.20,
    "lost_reverse_after": 0.75,
    "lost_reverse_duration": 0.55,
    "lost_scan_period": 1.20,
    "lost_forward_pct": 11.0,
    "lost_reverse_pct": -8.0,
    "lost_turn_pct": 32.0,
}

PRESETS = {
    "默认稳态": {
        "control_mode": "pid", "kp": 38.0, "ki": 0.0, "kd": 5.0,
        "base_speed_pct": 32.0, "base_min_pct": 16.0, "curve_factor": 0.55,
        "threshold_mode": "otsu", "roi_top_ratio": 0.55, "roi_bottom_ratio": 0.95,
        "min_area": 180.0, "error_filter_alpha": 0.60,
        "lost_strategy": "arc_recovery", "lost_forward_pct": 11.0, "lost_turn_pct": 32.0,
    },
    "慢速保守": {
        "control_mode": "pd", "kp": 28.0, "ki": 0.0, "kd": 6.0,
        "base_speed_pct": 20.0, "base_min_pct": 12.0, "curve_factor": 0.65,
        "roi_top_ratio": 0.50, "roi_bottom_ratio": 0.98,
        "min_area": 140.0, "error_filter_alpha": 0.70,
        "lost_forward_pct": 8.0, "lost_turn_pct": 28.0,
    },
    "快速激进": {
        "control_mode": "pd", "kp": 48.0, "ki": 0.0, "kd": 7.5,
        "base_speed_pct": 46.0, "base_min_pct": 20.0, "curve_factor": 0.50,
        "roi_top_ratio": 0.58, "roi_bottom_ratio": 0.96,
        "min_area": 200.0, "error_filter_alpha": 0.45,
        "lost_forward_pct": 12.0, "lost_turn_pct": 36.0,
    },
    "弯道稳定": {
        "control_mode": "pd", "kp": 34.0, "ki": 0.0, "kd": 8.0,
        "base_speed_pct": 26.0, "base_min_pct": 13.0, "curve_factor": 0.80,
        "roi_top_ratio": 0.48, "roi_bottom_ratio": 0.98,
        "min_area": 150.0, "error_filter_alpha": 0.68,
        "lost_forward_pct": 9.0, "lost_turn_pct": 34.0,
    },
    "找线强化": {
        "control_mode": "pd", "kp": 32.0, "ki": 0.0, "kd": 6.0,
        "base_speed_pct": 22.0, "base_min_pct": 12.0, "curve_factor": 0.60,
        "roi_top_ratio": 0.45, "roi_bottom_ratio": 0.99,
        "min_area": 100.0, "error_filter_alpha": 0.62,
        "lost_strategy": "arc_recovery", "lost_forward_pct": 10.0, "lost_reverse_pct": -9.0,
        "lost_turn_pct": 40.0, "lost_scan_period": 0.90,
    },
}


class CameraTunerGUI(object):
    def __init__(self):
        rospy.init_node("camera_tuner_gui_node", anonymous=False)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.reset_pid_pub = rospy.Publisher("/line_follower/reset_pid", Empty, queue_size=1)

        self.latest = {
            "online": False,
            "error": 0.0,
            "steer": 0.0,
            "pid": [0.0, 0.0, 0.0],
            "wheel": [0.0, 0.0, 0.0],
            "state": "waiting...",
            "vision": "waiting...",
        }
        self._updating_widgets = False
        self.widgets = {}
        self.vars = {}

        rospy.Subscriber("/line_follower/online", Bool, lambda m: self._set_latest("online", bool(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/error", Float32, lambda m: self._set_latest("error", float(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/steer", Float32, lambda m: self._set_latest("steer", float(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/pid_terms", Float32MultiArray, lambda m: self._set_latest("pid", list(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/wheel_pwm", Float32MultiArray, lambda m: self._set_latest("wheel", list(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/state", String, lambda m: self._set_latest("state", str(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/vision_state", String, lambda m: self._set_latest("vision", str(m.data)), queue_size=1)

        self.root = tk.Tk()
        self.root.title("三轮小车摄像头巡线 — 可视化调参面板")
        self.root.geometry("1040x760")
        self.root.minsize(980, 680)
        self._build_style()
        self._build_ui()
        self._load_params_to_widgets()
        self._refresh_status_loop()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _set_latest(self, key, value):
        self.latest[key] = value

    def _build_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("Microsoft YaHei", 15, "bold"), foreground="#0d47a1")
        style.configure("Card.TLabelframe", padding=8)
        style.configure("Card.TLabelframe.Label", font=("Microsoft YaHei", 10, "bold"), foreground="#1565c0")
        style.configure("Big.TLabel", font=("Consolas", 18, "bold"))
        style.configure("State.TLabel", font=("Consolas", 10))

    def _build_ui(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)

        header = ttk.Frame(root, padding=(12, 8))
        header.grid(row=0, column=0, columnspan=2, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="三轮小车摄像头巡线 — HTML 风格实时调参", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="图像ROI → 黑线质心 → error → PID → 差速驱动", foreground="#607d8b").grid(row=1, column=0, sticky="w")

        btns = ttk.Frame(header)
        btns.grid(row=0, column=1, rowspan=2, sticky="e")
        self.enabled_var = tk.BooleanVar(value=bool(rospy.get_param("/enabled", PARAM_DEFAULTS["enabled"])))
        ttk.Button(btns, text="▶ 启动巡线", command=lambda: self._set_enabled(True)).grid(row=0, column=0, padx=3)
        ttk.Button(btns, text="⏸ 暂停停车", command=lambda: self._set_enabled(False)).grid(row=0, column=1, padx=3)
        ttk.Button(btns, text="🔄 重置仿真", command=self._reset_world).grid(row=0, column=2, padx=3)
        ttk.Button(btns, text="🧹 清空PID", command=self._reset_pid_by_toggle).grid(row=0, column=3, padx=3)

        left = ttk.Frame(root, padding=(12, 0, 6, 10))
        left.grid(row=1, column=0, sticky="nsew")
        right = ttk.Frame(root, padding=(6, 0, 12, 10))
        right.grid(row=1, column=1, sticky="nsew")
        for col in (left, right):
            col.columnconfigure(0, weight=1)

        self._build_control_card(left).grid(row=0, column=0, sticky="ew", pady=6)
        self._build_vision_card(left).grid(row=1, column=0, sticky="ew", pady=6)
        self._build_recovery_card(left).grid(row=2, column=0, sticky="ew", pady=6)

        self._build_status_card(right).grid(row=0, column=0, sticky="ew", pady=6)
        self._build_preset_card(right).grid(row=1, column=0, sticky="ew", pady=6)
        self._build_help_card(right).grid(row=2, column=0, sticky="ew", pady=6)

    def _make_card(self, parent, title):
        return ttk.LabelFrame(parent, text=title, style="Card.TLabelframe")

    def _param(self, name):
        return rospy.get_param("/" + name, PARAM_DEFAULTS[name])

    def _set_param(self, name, value):
        rospy.set_param("/" + name, value)

    def _add_slider(self, parent, row, name, label, from_, to, resolution=0.1, integer=False):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        var = tk.DoubleVar(value=float(self._param(name)))
        value_label = ttk.Label(parent, width=8, anchor="e", font=("Consolas", 10, "bold"))
        value_label.grid(row=row, column=2, sticky="e", padx=4)
        scale = ttk.Scale(parent, from_=from_, to=to, variable=var, orient="horizontal")
        scale.grid(row=row, column=1, sticky="ew", padx=4)
        parent.columnconfigure(1, weight=1)

        def fmt(v):
            return str(int(round(v))) if integer else ("%.3f" % v if abs(to - from_) <= 2 else "%.2f" % v)

        def on_change(_=None):
            if self._updating_widgets:
                return
            v = float(var.get())
            if integer:
                v = int(round(v))
            # Make OpenCV odd kernel sliders usable even if the user stops on an even value.
            if name in ("blur_kernel", "morph_kernel"):
                v = max(1, int(round(v)))
                if v > 1 and v % 2 == 0:
                    v += 1
            value_label.configure(text=fmt(float(v)))
            self._set_param(name, v)

        var.trace_add("write", lambda *_: on_change())
        value_label.configure(text=fmt(float(var.get())))
        self.vars[name] = var
        self.widgets[name] = (scale, value_label)
        return scale

    def _add_combo(self, parent, row, name, label, values):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        var = tk.StringVar(value=str(self._param(name)))
        combo = ttk.Combobox(parent, textvariable=var, values=values, state="readonly", width=16)
        combo.grid(row=row, column=1, sticky="ew", padx=4, pady=3)
        combo.bind("<<ComboboxSelected>>", lambda _e: self._set_param(name, var.get()))
        parent.columnconfigure(1, weight=1)
        self.vars[name] = var
        self.widgets[name] = combo
        return combo

    def _build_control_card(self, parent):
        card = self._make_card(parent, "① PID 与差速驱动")
        self._add_combo(card, 0, "control_mode", "控制模式", ["p", "pd", "pi", "pid"])
        self._add_slider(card, 1, "kp", "Kp", 0, 80, 0.1)
        self._add_slider(card, 2, "ki", "Ki", 0, 5, 0.01)
        self._add_slider(card, 3, "kd", "Kd", 0, 15, 0.01)
        self._add_slider(card, 4, "base_speed_pct", "基础速度%", 0, 80, 1)
        self._add_slider(card, 5, "base_min_pct", "弯道最低%", 0, 40, 1)
        self._add_slider(card, 6, "curve_factor", "弯道降速", 0, 1.5, 0.01)
        self._add_slider(card, 7, "pos_out_limit", "转向限幅%", 10, 80, 1)
        self._add_slider(card, 8, "max_linear", "最大线速度", 0.1, 1.2, 0.01)
        self._add_slider(card, 9, "max_angular", "最大角速度", 0.5, 5.0, 0.01)
        return card

    def _build_vision_card(self, parent):
        card = self._make_card(parent, "② 摄像头识别参数")
        self._add_combo(card, 0, "threshold_mode", "阈值模式", ["otsu", "fixed", "adaptive"])
        self._add_slider(card, 1, "roi_top_ratio", "ROI 上边界", 0.20, 0.90, 0.01)
        self._add_slider(card, 2, "roi_bottom_ratio", "ROI 下边界", 0.40, 1.00, 0.01)
        self._add_slider(card, 3, "roi_left_ratio", "ROI 左边界", 0.00, 0.40, 0.01)
        self._add_slider(card, 4, "roi_right_ratio", "ROI 右边界", 0.60, 1.00, 0.01)
        self._add_slider(card, 5, "black_threshold", "固定阈值", 0, 255, 1, integer=True)
        self._add_slider(card, 6, "min_area", "最小轮廓面积", 20, 1000, 1)
        self._add_slider(card, 7, "min_black_ratio", "最小黑色占比", 0.000, 0.020, 0.0005)
        self._add_slider(card, 8, "error_filter_alpha", "误差滤波", 0.0, 0.95, 0.01)
        self._add_slider(card, 9, "blur_kernel", "模糊核", 1, 15, 2, integer=True)
        self._add_slider(card, 10, "morph_kernel", "形态学核", 1, 15, 2, integer=True)
        return card

    def _build_recovery_card(self, parent):
        card = self._make_card(parent, "③ 丢线恢复参数")
        self._add_combo(card, 0, "lost_strategy", "恢复策略", ["arc_recovery", "spin", "stop"])
        self._add_slider(card, 1, "lost_grace_time", "前弧搜索时间", 0.0, 1.5, 0.01)
        self._add_slider(card, 2, "lost_reverse_after", "倒车触发时间", 0.1, 2.5, 0.01)
        self._add_slider(card, 3, "lost_reverse_duration", "倒车持续时间", 0.0, 1.5, 0.01)
        self._add_slider(card, 4, "lost_scan_period", "左右扫描周期", 0.3, 3.0, 0.01)
        self._add_slider(card, 5, "lost_forward_pct", "丢线前进%", 0, 30, 1)
        self._add_slider(card, 6, "lost_reverse_pct", "丢线倒车%", -25, 0, 1)
        self._add_slider(card, 7, "lost_turn_pct", "丢线转向%", 5, 60, 1)
        return card

    def _build_status_card(self, parent):
        card = self._make_card(parent, "实时状态")
        grid = ttk.Frame(card)
        grid.grid(row=0, column=0, sticky="ew")
        for i in range(4):
            grid.columnconfigure(i, weight=1)
        self.status_labels = {}
        items = [("online", "在线"), ("error", "error"), ("steer", "steer"), ("base", "base")]
        for i, (key, text) in enumerate(items):
            box = ttk.Frame(grid, padding=6, relief="groove")
            box.grid(row=0, column=i, sticky="nsew", padx=3, pady=3)
            ttk.Label(box, text=text, foreground="#607d8b").pack()
            lab = ttk.Label(box, text="--", style="Big.TLabel")
            lab.pack()
            self.status_labels[key] = lab

        self.status_labels["pid"] = ttk.Label(card, text="P: --    I: --    D: --", style="State.TLabel")
        self.status_labels["pid"].grid(row=1, column=0, sticky="w", padx=4, pady=4)
        self.status_labels["wheel"] = ttk.Label(card, text="左轮: --    右轮: --", style="State.TLabel")
        self.status_labels["wheel"].grid(row=2, column=0, sticky="w", padx=4, pady=4)
        self.status_labels["state"] = ttk.Label(card, text="controller: --", wraplength=470, foreground="#455a64")
        self.status_labels["state"].grid(row=3, column=0, sticky="w", padx=4, pady=4)
        self.status_labels["vision"] = ttk.Label(card, text="vision: --", wraplength=470, foreground="#455a64")
        self.status_labels["vision"].grid(row=4, column=0, sticky="w", padx=4, pady=4)
        return card

    def _build_preset_card(self, parent):
        card = self._make_card(parent, "参数预设")
        for i, name in enumerate(PRESETS.keys()):
            ttk.Button(card, text=name, command=lambda n=name: self._apply_preset(n)).grid(row=i // 2, column=i % 2, sticky="ew", padx=4, pady=4)
        card.columnconfigure(0, weight=1)
        card.columnconfigure(1, weight=1)
        return card

    def _build_help_card(self, parent):
        card = self._make_card(parent, "调参提示")
        msg = (
            "1. 先用“慢速保守”让车稳定在线，再逐步提高基础速度。\n"
            "2. 弯道冲出线：降低基础速度，提高 Kd 或弯道降速。\n"
            "3. 来回摆动：Kp 偏大或滤波太小，降低 Kp / 增大误差滤波。\n"
            "4. 看不见黑线：调整 ROI 上/下边界、min_area，或切换 fixed/adaptive。\n"
            "5. 丢线回不来：使用找线强化，增大丢线转向或缩短扫描周期。\n"
            "图像调试请看 RViz 的 /line_follower/debug_image 或运行 rqt_image_view。"
        )
        ttk.Label(card, text=msg, justify="left", wraplength=480).grid(row=0, column=0, sticky="w", padx=4, pady=4)
        return card

    def _load_params_to_widgets(self):
        self._updating_widgets = True
        try:
            for name, var in self.vars.items():
                if name not in PARAM_DEFAULTS:
                    continue
                v = self._param(name)
                if isinstance(var, tk.StringVar):
                    var.set(str(v))
                else:
                    try:
                        var.set(float(v))
                    except Exception:
                        pass
        finally:
            self._updating_widgets = False

    def _set_widgets_from_params(self, updates):
        self._updating_widgets = True
        try:
            for name, value in updates.items():
                self._set_param(name, value)
                var = self.vars.get(name)
                if var is None:
                    continue
                if isinstance(var, tk.StringVar):
                    var.set(str(value))
                else:
                    try:
                        var.set(float(value))
                    except Exception:
                        pass
        finally:
            self._updating_widgets = False
        # Update value labels manually because traces are muted while applying presets.
        for name, value in updates.items():
            if name in self.vars and not isinstance(self.vars[name], tk.StringVar):
                v = float(self.vars[name].get())
                self._set_param(name, v)
                widget = self.widgets.get(name)
                if isinstance(widget, tuple) and len(widget) == 2:
                    label = widget[1]
                    text = str(int(round(v))) if name in ("black_threshold", "blur_kernel", "morph_kernel") else ("%.3f" % v if abs(v) < 2 else "%.2f" % v)
                    label.configure(text=text)

    def _apply_preset(self, name):
        updates = PRESETS.get(name, {})
        self._set_widgets_from_params(updates)
        rospy.loginfo("Applied camera line follower preset: %s", name)

    def _set_enabled(self, enabled):
        self._set_param("enabled", bool(enabled))
        if not enabled:
            self.cmd_pub.publish(Twist())
        rospy.loginfo("line follower enabled=%s", enabled)

    def _reset_pid_by_toggle(self):
        self.reset_pid_pub.publish(Empty())
        rospy.loginfo("PID reset requested from tuner GUI")

    def _reset_world(self):
        def worker():
            try:
                rospy.wait_for_service("/gazebo/reset_world", timeout=2.0)
                srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
                srv()
                self._set_enabled(True)
                rospy.loginfo("Gazebo world reset requested from tuner GUI")
            except Exception as exc:
                rospy.logwarn("Failed to call /gazebo/reset_world: %s", exc)
                self.root.after(0, lambda: messagebox.showwarning("重置失败", "未能调用 /gazebo/reset_world。请确认 Gazebo 已启动。"))
        threading.Thread(target=worker, daemon=True).start()

    def _refresh_status_loop(self):
        online = bool(self.latest.get("online", False))
        enabled = bool(rospy.get_param("/enabled", True))
        err = float(self.latest.get("error", 0.0))
        steer = float(self.latest.get("steer", 0.0))
        wheel = self.latest.get("wheel", [0.0, 0.0, 0.0])
        pid = self.latest.get("pid", [0.0, 0.0, 0.0])
        while len(wheel) < 3:
            wheel.append(0.0)
        while len(pid) < 3:
            pid.append(0.0)

        if online and enabled:
            online_text, fg = "● 在线", "#2e7d32"
        elif not enabled:
            online_text, fg = "⏸ 暂停", "#f57f17"
        else:
            online_text, fg = "✗ 丢线", "#c62828"
        self.status_labels["online"].configure(text=online_text, foreground=fg)
        self.status_labels["error"].configure(text="%+.3f" % err, foreground="#ef6c00")
        self.status_labels["steer"].configure(text="%+.1f" % steer, foreground="#1565c0")
        self.status_labels["base"].configure(text="%.1f%%" % float(wheel[2]), foreground="#00695c")
        self.status_labels["pid"].configure(text="P: %+.2f    I: %+.2f    D: %+.2f" % (pid[0], pid[1], pid[2]))
        self.status_labels["wheel"].configure(text="左轮: %+.1f%%    右轮: %+.1f%%" % (wheel[0], wheel[1]))
        self.status_labels["state"].configure(text="controller: " + str(self.latest.get("state", "--")))
        self.status_labels["vision"].configure(text="vision: " + str(self.latest.get("vision", "--")))
        if not rospy.is_shutdown():
            self.root.after(100, self._refresh_status_loop)

    def _on_close(self):
        try:
            self.root.destroy()
        except Exception:
            pass
        rospy.signal_shutdown("camera tuner GUI closed")

    def spin(self):
        self.root.mainloop()


def main():
    gui = CameraTunerGUI()
    gui.spin()


if __name__ == "__main__":
    main()
