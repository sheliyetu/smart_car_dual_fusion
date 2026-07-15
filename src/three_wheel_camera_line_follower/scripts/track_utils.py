#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared track geometry for Gazebo visual track and virtual photoelectric sensors.

Coordinate convention: ROS/Gazebo map plane, x forward, y left, z up.
The track is a closed Catmull-Rom curve adapted from the HTML canvas simulator.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]


def catmull_rom_closed(points: Sequence[Point], samples_per_seg: int = 12) -> List[Point]:
    dense: List[Point] = []
    n = len(points)
    for i in range(n):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]
        for j in range(samples_per_seg):
            t = float(j) / float(samples_per_seg)
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * (
                (2.0 * p1[0])
                + (-p0[0] + p2[0]) * t
                + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                (2.0 * p1[1])
                + (-p0[1] + p2[1]) * t
                + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
            )
            dense.append((x, y))
    return dense


TRACK_PARAMS = {
    "default": {  # original: medium ellipse + multi-frequency disturbance
        "rx_base": 2.10, "ry_base": 1.42,
        "rx_a1": 0.23, "rx_f1": 3, "rx_a2": 0.11, "rx_f2": 5,
        "ry_a1": 0.19, "ry_f1": 2, "ry_a2": 0.15, "ry_f2": 4,
    },
    "oval": {  # irregular oval, 3 freq components
        "rx_base": 2.60, "ry_base": 1.50,
        "rx_a1": 0.18, "rx_f1": 2, "rx_a2": 0.10, "rx_f2": 5, "rx_a3": 0.05, "rx_f3": 7,
        "ry_a1": 0.15, "ry_f1": 3, "ry_a2": 0.08, "ry_f2": 4, "ry_a3": 0.04, "ry_f3": 6,
    },
    "sine": {  # irregular winding, varied curve radii
        "rx_base": 2.40, "ry_base": 1.50,
        "rx_a1": 0.25, "rx_f1": 2, "rx_a2": 0.12, "rx_f2": 4, "rx_a3": 0.06, "rx_f3": 6,
        "ry_a1": 0.20, "ry_f1": 2, "ry_a2": 0.10, "ry_f2": 5, "ry_a3": 0.05, "ry_f3": 7,
    },
    "circle": {  # large circle, radius ~2.5m
        "rx_base": 2.50, "ry_base": 2.50,
        "rx_a1": 0.05, "rx_f1": 3,
        "ry_a1": 0.05, "ry_f1": 3,
    },
}


def _rx(t, p):
    v = p["rx_base"]
    v += p.get("rx_a1",0) * math.sin(p.get("rx_f1",0)*t) + p.get("rx_a2",0) * math.cos(p.get("rx_f2",0)*t)
    v += p.get("rx_a3",0) * math.sin(p.get("rx_f3",0)*t) + p.get("rx_a4",0) * math.cos(p.get("rx_f4",0)*t)
    return v

def _ry(t, p):
    v = p["ry_base"]
    v += p.get("ry_a1",0) * math.cos(p.get("ry_f1",0)*t) - p.get("ry_a2",0) * math.sin(p.get("ry_f2",0)*t)
    v += p.get("ry_a3",0) * math.cos(p.get("ry_f3",0)*t) - p.get("ry_a4",0) * math.sin(p.get("ry_f4",0)*t)
    return v

def generate_track(track_type: str = "default", samples_per_seg: int = 48,
                   reverse: bool = False) -> List[Point]:
    p = TRACK_PARAMS.get(track_type, TRACK_PARAMS["default"])
    raw: List[Point] = []
    n = 24
    for i in range(n):
        t = (float(i) / float(n)) * math.tau
        raw.append((_rx(t, p) * math.cos(t), _ry(t, p) * math.sin(t)))
    pts = catmull_rom_closed(raw, samples_per_seg=samples_per_seg)
    if reverse:
        pts = list(reversed(pts))
    return pts


def start_pose(sensor_forward: float = 0.18, track_type: str = "default") -> Tuple[float, float, float]:
    """Return robot spawn pose so the sensor bar starts on the first track point."""
    pts = generate_track(track_type=track_type, samples_per_seg=12)
    p0 = pts[0]
    p1 = pts[1]
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    x = p0[0] - math.cos(yaw) * sensor_forward
    y = p0[1] - math.sin(yaw) * sensor_forward
    return x, y, yaw


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    qx = ax + u * abx
    qy = ay + u * aby
    return math.hypot(px - qx, py - qy)


def min_distance_to_track(px: float, py: float, track: Sequence[Point]) -> float:
    best = float("inf")
    n = len(track)
    for i in range(n):
        ax, ay = track[i]
        bx, by = track[(i + 1) % n]
        d = point_to_segment_distance(px, py, ax, ay, bx, by)
        if d < best:
            best = d
    return best


def iter_segments(track: Sequence[Point]):
    n = len(track)
    for i in range(n):
        ax, ay = track[i]
        bx, by = track[(i + 1) % n]
        mx = 0.5 * (ax + bx)
        my = 0.5 * (ay + by)
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        yield i, ax, ay, bx, by, mx, my, length, yaw
