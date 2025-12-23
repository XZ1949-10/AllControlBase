cmake_minimum_required(VERSION 3.0.2)
project(controller_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# 生成消息
add_message_files(
  FILES
  LocalTrajectoryV4.msg
  UnifiedCmd.msg
  DiagnosticsV2.msg
  AttitudeCmd.msg
)

# 生成服务
add_service_files(
  FILES
  SetControllerState.srv
  GetDiagnostics.srv
  SetHoverYaw.srv
  GetAttitudeRateLimits.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 启用 Python 模块安装 (setup.py)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros message_runtime
)

# 安装 Python 模块
catkin_install_python(PROGRAMS
  scripts/controller_node.py
  scripts/dashboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 和 config
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
