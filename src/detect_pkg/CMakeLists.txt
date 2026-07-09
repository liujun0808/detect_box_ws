cmake_minimum_required(VERSION 3.8)
project(detect_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(upper_limb_interface REQUIRED)
find_package(Eigen3 REQUIRED)

# 使用 OpenCV aruco 模块的旧接口：
#   cv::aruco::detectMarkers()
#   cv::aruco::estimatePoseSingleMarkers()
# 不使用 objdetect/ArucoDetector，避免与你当前环境中的 OpenCV 库产生链接不匹配。
find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  calib3d
  imgcodecs
  aruco
)

message(STATUS "OpenCV_VERSION=${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
message(STATUS "OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")

add_executable(detect_server_node
  src/detect_sever_node.cpp
)

target_include_directories(detect_server_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(detect_server_node
  geometry_msgs
  rclcpp
  sensor_msgs
  upper_limb_interface
)

target_link_libraries(detect_server_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  opencv_aruco
  opencv_calib3d
  opencv_imgcodecs
  opencv_imgproc
  opencv_core
)

add_executable(detect_client_node
  src/detect_client_node.cpp
)

ament_target_dependencies(detect_client_node
  rclcpp
  upper_limb_interface
)

install(TARGETS
  detect_server_node
  detect_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/print_box_pose_in_pelvis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
