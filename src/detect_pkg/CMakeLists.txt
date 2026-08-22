cmake_minimum_required(VERSION 3.8)
project(detect_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(upper_limb_interface REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(detect_client_node
  src/detect_client_node.cpp
)

ament_target_dependencies(detect_client_node
  rclcpp
  upper_limb_interface
)

install(TARGETS
  detect_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/detect_server_node
  scripts/print_box_pose_in_pelvis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
