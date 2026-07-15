# 三轮差速小车双模循迹融合系统

基于 ROS1 Noetic + Gazebo 仿真，融合**摄像头视觉处理**与**虚拟五路光电传感器**的双模循迹控制系统。课程项目，作者：许德新。

---

## 快速开始

```bash
# 1. 解压
tar xzf smart_car_dual_fusion.tar.gz
cd smart_car_dual_fusion

# 2. 编译
catkin_make
source devel/setup.bash

# 3. 启动仿真（合成图像模式，不需要独立显卡）
roslaunch three_wheel_camera_line_follower demo.launch

# 4. 打开 Web 调参面板
#    浏览器 → http://127.0.0.1:8091/
```

首次启动后耐心等待 10-15 秒，Gazebo 加载完成后小车会自动开始循迹。

---

## 系统架构

```
                     ┌→ 摄像头检测器 → camera_error ─┐
                     │                                 │
Gazebo ─── 小车位姿 ─┼→ 光电传感器 → sensor_error ─┼→ 固定加权融合 → PID → cmd_vel
                     │                                 │
                     └→ 赛道几何 → 合成图像(备用) ────┘
```

双传感器各有独立置信度，任一掉线时自动降权，不会互相污染。

---

## 启动参数

```bash
# 合成图像模式（默认，不需要GPU/OpenGL）
roslaunch three_wheel_camera_line_follower demo.launch synthetic:=true

# 真实摄像头模式（需要支持 OpenGL 的 GPU 驱动）
roslaunch three_wheel_camera_line_follower demo.launch synthetic:=false

# 只开 Web 面板，关 RViz
roslaunch three_wheel_camera_line_follower demo.launch rviz:=false
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `synthetic` | `true` | true=合成图像，false=真实摄像头 |
| `gui` | `true` | Gazebo GUI |
| `rviz` | `true` | RViz 可视化 |
| `tuner_web` | `true` | Web 调参面板（8091端口） |
| `dashboard` | `true` | 终端仪表盘 |

---

## 文件结构

```
smart_car_dual_fusion/
├── src/three_wheel_camera_line_follower/
│   ├── scripts/
│   │   ├── camera_line_detector_node.py   # 摄像头 OpenCV 视觉处理
│   │   ├── line_sensor_node.py            # 虚拟五路光电传感器
│   │   ├── sensor_fusion_node.py          # 固定加权融合
│   │   ├── camera_line_follower_node.py   # PID 控制器 + 丢线恢复
│   │   ├── camera_synthetic_node.py       # 合成图像节点（GPU替代方案）
│   │   ├── camera_tuner_web_node.py       # Web 调参 HTTP 服务
│   │   ├── camera_tuner_gui_node.py       # Tkinter 调参面板
│   │   ├── camera_dashboard_node.py       # 终端仪表盘
│   │   ├── line_visualizer_node.py        # RViz 可视化
│   │   ├── make_line_track_world.py       # 赛道世界生成器
│   │   └── track_utils.py                # 赛道几何（Catmull-Rom）
│   ├── launch/
│   │   ├── demo.launch                    # 快速启动入口
│   │   ├── gazebo_camera_line_follower.launch  # 主启动文件
│   │   └── rviz_camera_line_follower.launch    # 仅 RViz
│   ├── urdf/three_wheel_camera_car.urdf.xacro  # 小车模型
│   ├── config/
│   │   ├── camera_line_follower.yaml     # PID/阈值/丢线参数
│   │   └── camera_line_follower.rviz     # RViz 布局
│   └── worlds/line_track_camera.world    # Gazebo 赛道
├── 技术报告.md                          # 完整技术文档
└── README.md                            # 本文件
```

---

## 各模块说明

### 摄像头视觉处理 (`camera_line_detector_node.py`)

订阅 `/camera/image_raw`，OpenCV 处理管线：

```
ROI裁剪(55%-95%高度) → 灰度 → 5×5高斯模糊 → Otsu阈值 →
5×5椭圆形态学开闭 → 轮廓筛选(面积>180) → 加权质心(底部权重×4) →
error = (cx − roi_w/2) / (roi_w/2) × 2.0
```

加权质心使底部（近车端）的线位置权重大 4 倍，提前感知弯道走向。

### 虚拟光电传感器 (`line_sensor_node.py`)

模拟 5 路光电传感器（ADC 0-4095），从赛道几何直接采样：

- 5 个传感器间距 9mm，位于车前方 180mm
- 白底 ADC ≈ 3500，黑线 ADC ≈ 800，随机噪声 ±30
- 二值化重心法计算偏差（支持 binary / analog / interp 三种算法）
- 对比度 < 300 判丢线

### 传感器融合 (`sensor_fusion_node.py`)

```python
error = (0.6 × camera_error + 0.4 × sensor_error) / (0.6 + 0.4)
# 任一传感器掉线(online=False)，自动移除其权重
```

固定加权融合，逻辑简单可靠，适合课程展示。

### PID 控制器 (`camera_line_follower_node.py`)

```python
steer = Kp × error + Ki × ∫error·dt + Kd × d(error)/dt
left_wheel  = base + steer
right_wheel = base - steer
base = clamp(base_speed - |steer| × curve_factor, base_min, base_speed)
```

- 支持 P/PD/PI/PID 四种模式切换
- 弯道自动减速（`curve_factor` 控制减速力度）
- 丢线恢复状态机：前弧搜索(0.75s) → 倒车弧(0.55s) → 交替扫描(1.2s) → 安全停止(超时)

### 合成图像模式 (`camera_synthetic_node.py`)

用于无 GPU 环境（如虚拟机）。从赛道几何实时生成 64×64 下视图像，
替代真实 Gazebo 摄像头，再经 OpenCV 同款阈值→质心处理。
**VM 上默认启用，无需独立显卡。**

---

## 调参

### Web 面板（推荐）
http://127.0.0.1:8091/

可调节：

| 分类 | 参数 |
|------|------|
| PID | Kp, Ki, Kd, 控制模式 |
| 速度 | base_speed, max_linear, curve_factor |
| 摄像头 | 阈值模式, ROI范围, 形态学核大小 |
| 丢线 | 各阶段持续时间/速度 |
| 融合 | camera_weight, sensor_weight |

### 命令行
```bash
# 查看当前参数
rosparam get /kp
rosparam get /base_speed_pct

# 动态调参
rosparam set /kp 20.0
rosparam set /base_speed_pct 50.0
```

### RViz
```bash
rosrun rviz rviz -d src/three_wheel_camera_line_follower/config/camera_line_follower.rviz
```

---

## 赛道自定义

修改 `scripts/make_line_track_world.py` 中的参数可生成不同赛道：

```python
# 例如：大椭圆赛道（更缓和的弯道）
track2_params = dict(rx_base=2.60, ry_base=1.50,
                     rx_amp1=0.15, rx_freq1=2,
                     ry_amp1=0.12, ry_freq1=2)
```

运行 `rosrun three_wheel_camera_line_follower make_line_track_world.py` 生成新的 `.world` 文件。

---

## 常见问题

### Gazebo 启动慢
首次启动需下载模型，耐心等待。后续启动会缓存。

### 摄像头不渲染 / error 始终为 0
当前处于合成图像模式（默认），不依赖真实摄像头。
如果需要真实摄像头模式，确保宿主机有 OpenGL 支持，设置 `synthetic:=false`。

### 小车不动 / 丢线
- 检查 Web 面板 "在线" 状态
- 增大 `base_speed_pct` 或降低 `curve_factor`
- 检查 `kp` 是否过低（建议 20-40）

### 编译报错 `cv_bridge` 找不到
```bash
sudo apt install ros-noetic-cv-bridge python3-opencv
```

---

## 致谢

基于哈尔滨工程大学许德新老师提供的 ROS 仿真项目框架。
