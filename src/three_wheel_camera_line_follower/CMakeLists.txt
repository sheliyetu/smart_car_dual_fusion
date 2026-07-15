cmake_minimum_required(VERSION 3.0.2)
project(three_wheel_camera_line_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera_synthetic_node.py
  scripts/camera_line_detector_node.py
  scripts/camera_line_follower_node.py
  scripts/camera_dashboard_node.py
  scripts/camera_tuner_gui_node.py
  scripts/camera_tuner_web_node.py
  scripts/line_visualizer_node.py
  scripts/make_line_track_world.py
  scripts/line_sensor_node.py
  scripts/sensor_fusion_node.py
  scripts/track_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
