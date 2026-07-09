cmake_minimum_required(VERSION 3.5)
project(upper_limb_interface)

# 接口包：只负责生成 msg/srv/action，不写业务代码。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 生成 UpperLimbManipulate.action 对应的 C++ / Python 接口代码。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/UpperLimbManipulate.action"
  "srv/DetectAprilTag.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
