#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate worlds/line_track_camera.world from the shared Catmull-Rom track."""
from __future__ import annotations

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from track_utils import generate_track  # noqa: E402


def segment_models(track, line_width=0.10):
    out = []
    for i in range(len(track)):
        x1, y1 = track[i]
        x2, y2 = track[(i + 1) % len(track)]
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-5:
            continue
        yaw = math.atan2(dy, dx)
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        out.append((i, cx, cy, yaw, length, line_width))
    return out


def make_world():
    track = generate_track(samples_per_seg=12)
    parts = []
    parts.append('<?xml version="1.0" ?>')
    parts.append('<sdf version="1.6">')
    parts.append('  <world name="camera_line_following_world">')
    parts.append('    <physics name="default_physics" type="ode">')
    parts.append('      <max_step_size>0.001</max_step_size>')
    parts.append('      <real_time_factor>1.0</real_time_factor>')
    parts.append('      <real_time_update_rate>1000</real_time_update_rate>')
    parts.append('    </physics>')
    parts.append('')
    parts.append('    <scene>')
    parts.append('      <ambient>0.9 0.9 0.9 1</ambient>')
    parts.append('      <background>0.86 0.90 0.94 1</background>')
    parts.append('      <shadows>false</shadows>')
    parts.append('    </scene>')
    parts.append('')
    parts.append('    <include><uri>model://sun</uri></include>')
    parts.append('')
    parts.append('    <light name="soft_area_light" type="point">')
    parts.append('      <pose>0 0 3.0 0 0 0</pose>')
    parts.append('      <diffuse>0.9 0.9 0.9 1</diffuse>')
    parts.append('      <specular>0.2 0.2 0.2 1</specular>')
    parts.append('      <attenuation><range>8</range><constant>0.7</constant><linear>0.08</linear><quadratic>0.01</quadratic></attenuation>')
    parts.append('    </light>')
    parts.append('')
    parts.append('    <model name="white_ground">')
    parts.append('      <static>true</static>')
    parts.append('      <link name="ground_link">')
    parts.append('        <collision name="ground_collision">')
    parts.append('          <geometry><box><size>6.0 4.5 0.02</size></box></geometry>')
    parts.append('          <surface><friction><ode><mu>1.4</mu><mu2>1.4</mu2></ode></friction></surface>')
    parts.append('        </collision>')
    parts.append('        <visual name="ground_visual">')
    parts.append('          <geometry><box><size>6.0 4.5 0.02</size></box></geometry>')
    parts.append('          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>')
    parts.append('        </visual>')
    parts.append('      </link>')
    parts.append('    </model>')
    parts.append('')
    parts.append('    <!-- Black closed track, visible to the Gazebo camera. -->')
    for i, cx, cy, yaw, length, width in segment_models(track):
        parts.append(f'    <model name="black_line_{i:03d}">')
        parts.append('      <static>true</static>')
        parts.append(f'      <pose>{cx:.6f} {cy:.6f} 0.018 0 0 {yaw:.6f}</pose>')
        parts.append('      <link name="line_link">')
        parts.append('        <visual name="line_visual">')
        parts.append(f'          <geometry><box><size>{length:.6f} {width:.6f} 0.006</size></box></geometry>')
        parts.append('          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular></material>')
        parts.append('        </visual>')
        parts.append('      </link>')
        parts.append('    </model>')
    parts.append('  </world>')
    parts.append('</sdf>')
    return '\n'.join(parts) + '\n'


if __name__ == '__main__':
    out_path = os.path.join(PKG_DIR, 'worlds', 'line_track_camera.world')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(make_world())
    print(out_path)
