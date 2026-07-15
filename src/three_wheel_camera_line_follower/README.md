# 三轮小车摄像头巡线 ROS1 Noetic 仿真（Web 可视化调参版）

本版本在 Gazebo + RViz 摄像头巡线基础上新增浏览器调参面板，避免 Tkinter 在 SSH、远程桌面或无 DISPLAY 环境中崩溃。启动后浏览器访问：

```bash
http://127.0.0.1:8091/
```

如果浏览器在另一台电脑，使用：

```bash
http://机器人电脑IP:8091/
```

Web 面板支持实时调整 PID、速度、ROI、阈值、OpenCV 形态学参数和丢线恢复策略，并直接预览 `/line_follower/debug_image`，可看到 ROI、黑线轮廓、质心点和误差线。

运行：

```bash
roslaunch three_wheel_camera_line_follower demo.launch
```

只跑 Gazebo + RViz，不开调参面板：

```bash
roslaunch three_wheel_camera_line_follower demo.launch tuner_web:=false tuner_gui:=false
```

仍想使用 Tkinter 面板：

```bash
roslaunch three_wheel_camera_line_follower demo.launch tuner_web:=false tuner_gui:=true
```

---

# three_wheel_camera_line_follower

ROS1 Noetic 三轮小车摄像头巡线仿真功能包。它保留三轮差速底盘、白底黑线赛道、PID 差速控制和 RViz/Gazebo 联动，但把原来的 5 路光电采样改为真实 Gazebo 摄像头图像处理。

## 1. 功能链路

原光电巡线：

```text
5 路 ADC -> 阈值/二值化 -> 误差 -> PID -> 左右轮差速
```

摄像头巡线：

```text
/camera/image_raw
  -> ROI 裁剪
  -> 灰度 + 模糊
  -> 黑线阈值分割 Otsu/fixed/adaptive
  -> 形态学去噪
  -> 最大轮廓/加权质心
  -> 归一化横向偏差 error [-1, 1]
  -> PID steer
  -> /cmd_vel
  -> Gazebo 差速驱动
```

其中 `error > 0` 表示黑线中心在图像右侧；ROS 中 `angular.z < 0` 表示右转，因此默认 `turn_sign=-1.0`。

## 2. 安装依赖

Ubuntu 20.04 + ROS Noetic：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros \
  ros-noetic-xacro \
  ros-noetic-cv-bridge \
  ros-noetic-image-view \
  python3-opencv
```

## 3. 编译运行

```bash
mkdir -p ~/catkin_ws/src
cp -r three_wheel_camera_line_follower ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch three_wheel_camera_line_follower demo.launch
```

如果只想开 Gazebo，不开 RViz：

```bash
roslaunch three_wheel_camera_line_follower gazebo_camera_line_follower.launch rviz:=false
```

如果希望单独弹出 OpenCV 调试图像窗口：

```bash
roslaunch three_wheel_camera_line_follower gazebo_camera_line_follower.launch image_view:=true
```

也可以手动查看图像：

```bash
rqt_image_view /line_follower/debug_image
```

## 4. Gazebo 与 RViz 中能看到什么

Gazebo：

- 白色地面；
- 黑色闭合巡线赛道；
- 三轮差速小车；
- 前置下倾摄像头；
- 小车自动沿黑线巡线。

RViz：

- `RobotModel` 三轮小车；
- `/line_follower/track_markers` 黑线赛道；
- `/line_follower/camera_markers` 摄像头视场、ROI 控制条、误差箭头、状态文本；
- `/odom` 运动轨迹；
- `/camera/image_raw` 原始相机图像；
- `/line_follower/debug_image` OpenCV 处理结果，含 ROI、中心线、黑线轮廓、质心和误差文本。

## 5. 主要话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo 摄像头原图 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 摄像头参数 |
| `/line_follower/debug_image` | `sensor_msgs/Image` | ROI/轮廓/质心/误差可视化 |
| `/line_follower/mask_image` | `sensor_msgs/Image` | 黑线二值掩膜 |
| `/line_follower/error` | `std_msgs/Float32` | 归一化横向误差，范围约 [-1, 1] |
| `/line_follower/online` | `std_msgs/Bool` | 是否识别到黑线 |
| `/line_follower/steer` | `std_msgs/Float32` | PID 输出转向百分比 |
| `/line_follower/pid_terms` | `std_msgs/Float32MultiArray` | P/I/D 分量 |
| `/line_follower/wheel_pwm` | `std_msgs/Float32MultiArray` | 左轮、右轮、基础速度百分比 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发给 Gazebo 差速驱动插件 |
| `/odom` | `nav_msgs/Odometry` | 小车里程计 |

## 6. 运行时调参

### PID 调参

```bash
rosparam set /control_mode pid   # p | pd | pi | pid
rosparam set /kp 38.0
rosparam set /ki 0.0
rosparam set /kd 5.0
rosparam set /base_speed_pct 32.0
rosparam set /pos_out_limit 60.0
```

推荐课堂演示：

```bash
# 只用 P，能看出弯道抖动或滞后
rosparam set /control_mode p
rosparam set /kp 50
rosparam set /kd 0

# PD，抑制振荡
rosparam set /control_mode pd
rosparam set /kp 38
rosparam set /kd 5

# 速度过高，观察丢线
rosparam set /base_speed_pct 55
```

### 图像识别调参

```bash
# 默认 Otsu 自动阈值，适合白底黑线
rosparam set /threshold_mode otsu

# 固定阈值，适合讲解阈值过高/过低造成误检
rosparam set /threshold_mode fixed
rosparam set /black_threshold 90

# 光照变化时可尝试自适应阈值
rosparam set /threshold_mode adaptive
rosparam set /adaptive_block_size 31
rosparam set /adaptive_c 7

# 调整 ROI：只看图像下半部分，越靠下越重视近处线
rosparam set /roi_top_ratio 0.55
rosparam set /roi_bottom_ratio 0.95
```

## 7. 节点说明

### `camera_line_detector_node.py`

负责图像处理。关键步骤：

1. 从 `/camera/image_raw` 读取图像；
2. 裁剪底部 ROI；
3. 灰度、模糊、黑线阈值分割；
4. 形态学开闭运算去噪；
5. 计算黑线最大轮廓和加权质心；
6. 发布 `/line_follower/error`、`/line_follower/online`、`/line_follower/debug_image`。

### `camera_line_follower_node.py`

负责控制。关键步骤：

1. 订阅视觉误差；
2. PID 计算转向量；
3. 弯道自动降速；
4. 生成 `/cmd_vel`；
5. 丢线时短时按最后方向搜索，超过阈值停止。

### `line_visualizer_node.py`

负责 RViz 可视化。发布赛道、误差箭头、摄像头视场和状态文字。

### `camera_dashboard_node.py`

终端实时显示在线状态、误差、PID 分量、左右轮速度。

## 8. 常见问题

### 1）Gazebo 中车不动

检查 `/cmd_vel` 是否有输出：

```bash
rostopic echo /cmd_vel
rostopic echo /line_follower/online
rostopic echo /line_follower/error
```

如果 `/line_follower/online` 一直是 `False`，说明图像没有识别到黑线，先看 `/line_follower/debug_image`。

### 2）识别不到黑线

依次尝试：

```bash
rosparam set /threshold_mode otsu
rosparam set /roi_top_ratio 0.50
rosparam set /roi_bottom_ratio 0.98
rosparam set /min_area 80
rosparam set /min_black_ratio 0.001
```

### 3）小车转向反了

把方向符号反过来：

```bash
rosparam set /turn_sign 1.0
```

默认设计中 `error > 0` 表示图像右侧，应该右转，所以 `turn_sign=-1.0`。

### 4）小车在弯道振荡

降低 `kp` 或增加 `kd`：

```bash
rosparam set /kp 28
rosparam set /kd 6
rosparam set /base_speed_pct 25
```

### 5）小车过弯丢线

降低速度，扩大 ROI 或提高弯道降速：

```bash
rosparam set /base_speed_pct 25
rosparam set /curve_factor 0.75
rosparam set /roi_top_ratio 0.45
```

## 9. 教学展示建议

这套摄像头巡线比 5 路光电巡线更适合讲“AI/视觉感知到控制”的完整链条：

1. 先展示 Gazebo 中的真实相机图像；
2. 再展示 `/line_follower/debug_image` 中的 ROI、黑线轮廓和质心；
3. 说明 `error = 黑线中心 - 图像中心`；
4. 修改 `kp/kd/base_speed_pct`，让学生观察振荡、滞后、丢线；
5. 对比光电巡线：光电是离散 5 点采样，摄像头是连续图像特征提取。


## HTML 风格可视化调参面板

新版 `demo.launch` 默认启动 `camera_tuner_gui_node.py`，提供类似 HTML 仿真的交互面板：

- PID 与差速驱动：`control_mode / Kp / Ki / Kd / base_speed_pct / curve_factor / max_linear / max_angular`
- 图像识别：`threshold_mode / ROI 上下左右边界 / black_threshold / min_area / error_filter_alpha / blur_kernel / morph_kernel`
- 丢线恢复：`lost_strategy / lost_forward_pct / lost_reverse_pct / lost_turn_pct / lost_scan_period`
- 状态显示：在线/丢线、error、steer、PID 三项、左右轮 PWM、视觉状态、控制器状态
- 操作按钮：启动巡线、暂停停车、重置 Gazebo、清空 PID、默认/慢速/快速/弯道/找线强化预设

如果系统缺少 Tkinter：

```bash
sudo apt update
sudo apt install -y python3-tk
```

如需关闭调参面板：

```bash
roslaunch three_wheel_camera_line_follower demo.launch tuner_gui:=false
```

注意：调参面板通过 `rosparam set` 实时写入全局参数，检测节点和控制节点会自动刷新，不需要重启仿真。

## 2026-06 Safety update: prevent the camera car from driving away

If the black line leaves the camera FOV, the controller now uses a safety guard:

- `lost_safe_stop_after`: after this many seconds without the line, stop blind recovery.
- `lost_auto_reset_after`: after this many seconds without the line, reset the Gazebo model to the start pose. Set to `0` to disable auto reset.
- `out_of_bounds_enable`: stop/reset if odometry leaves the demo area.
- `out_of_bounds_action`: `reset` or `stop`.
- `x_abs_limit`, `y_abs_limit`: world safety boundary in meters.

For classroom demo, keep the default values.  For debugging long recovery behavior, set:

```bash
rosparam set /lost_auto_reset_after 0
rosparam set /out_of_bounds_action stop
```
