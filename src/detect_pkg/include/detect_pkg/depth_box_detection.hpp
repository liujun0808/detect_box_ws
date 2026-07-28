#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <librealsense2/rs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/core.hpp>

namespace detect_pkg
{

enum class MotionState : std::uint8_t
{
  UNKNOWN = 0,
  MOVING = 1,
  STATIC_CANDIDATE = 2
};

enum class GeometryState : std::uint8_t
{
  NO_BOX = 0,
  PARTIAL_ENTERING = 1,
  PARTIAL_GEOMETRY = 2,
  ESTIMATED_CENTER = 3,
  SUFFICIENT_GEOMETRY = 4,
  GEOMETRY_FAILED = 5
};

enum class DetectionStatus : std::uint8_t
{
  NO_BOX = 0,
  MOVING_PARTIAL_ENTERING = 1,
  MOVING_PARTIAL_GEOMETRY = 2,
  MOVING_ESTIMATED_CENTER = 3,
  MOVING_SUFFICIENT_GEOMETRY = 4,
  MOTION_UNCERTAIN = 5,
  STATIC_CANDIDATE = 6,
  DETECTION_FAILED = 7
};

struct BoxModel
{
  float outer_length_m{0.600F};
  float outer_width_m{0.400F};
  float outer_height_m{0.280F};
  float rim_thickness_m{0.025F};
  float inner_length_m{0.550F};
  float inner_width_m{0.350F};
  float cavity_depth_m{0.220F};
};

struct DepthBoxDetectionConfig
{
  std::string service_name{"detect"};
  std::string camera_serial_no;
  int depth_width{640};
  int depth_height{480};
  int depth_fps{30};
  int camera_warmup_frames{15};
  int frame_timeout_ms{1000};
  double capture_window_sec{0.8};

  int motion_frame_gap{5};
  int min_motion_votes{3};
  float depth_difference_threshold_m{0.025F};
  int morph_close_size{7};
  int morph_open_size{3};
  int dilate_size{5};
  int min_motion_area_px{250};

  int candidate_expand_left_px{20};
  int candidate_expand_right_px{250};
  int candidate_expand_up_px{50};
  int candidate_expand_down_px{80};
  int border_margin_px{8};

  float roi_x_min_m{-0.80F};
  float roi_x_max_m{0.80F};
  float roi_y_min_m{-0.50F};
  float roi_y_max_m{1.20F};
  float roi_z_min_m{0.30F};
  float roi_z_max_m{2.00F};

  int point_stride_px{2};
  int min_candidate_points{300};
  int min_rim_candidate_points{80};
  float rim_image_top_ratio{0.65F};
  float near_depth_percentile{0.25F};
  float near_depth_band_m{0.18F};

  int plane_max_iterations{180};
  float plane_inlier_distance_m{0.018F};
  int plane_min_inliers{45};

  float min_pose_visible_length_ratio{0.45F};
  float min_pose_visible_width_ratio{0.45F};
  float sufficient_visible_length_ratio{0.75F};
  float sufficient_visible_width_ratio{0.75F};

  float moving_position_span_threshold_m{0.015F};
  float static_position_span_threshold_m{0.008F};

  BoxModel box_model;
  Eigen::Matrix4d camera_to_base{Eigen::Matrix4d::Identity()};
  std::string debug_output_dir{"debug_depth"};
};

struct DepthFrame
{
  cv::Mat depth_u16;
  double timestamp_sec{0.0};
  rs2_intrinsics intrinsics{};
  float depth_scale_m{0.001F};
  int sequence_index{-1};
};

struct PointSample
{
  Eigen::Vector3f point_camera{Eigen::Vector3f::Zero()};
  cv::Point pixel;
};

struct Plane3D
{
  Eigen::Vector3f normal{Eigen::Vector3f::Zero()};
  float d{0.0F};
  Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
  std::vector<std::size_t> inlier_indices;
  float rmse_m{0.0F};
};

struct PlaneBasis
{
  Eigen::Vector3f origin_camera{Eigen::Vector3f::Zero()};
  Eigen::Vector3f axis_u_camera{Eigen::Vector3f::UnitX()};
  Eigen::Vector3f axis_v_camera{Eigen::Vector3f::UnitY()};
  Eigen::Vector3f normal_camera{Eigen::Vector3f::UnitZ()};
};

struct GeometryEstimate
{
  bool valid{false};
  bool pose_detected{false};
  GeometryState geometry_state{GeometryState::GEOMETRY_FAILED};
  Eigen::Vector3f position_camera{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f rotation_camera_box{Eigen::Matrix3f::Identity()};
  float visible_length_m{0.0F};
  float visible_width_m{0.0F};
  float length_visible_ratio{0.0F};
  float width_visible_ratio{0.0F};
  float plane_rmse_m{0.0F};
  bool touches_right_border{false};
  bool touches_left_border{false};
  float confidence{0.0F};
};

struct MotionEstimate
{
  MotionState state{MotionState::UNKNOWN};
  float position_span_m{0.0F};
  float confidence{0.0F};
};

struct DetectionResult
{
  DetectionStatus status{DetectionStatus::DETECTION_FAILED};
  MotionState motion_state{MotionState::UNKNOWN};
  GeometryState geometry_state{GeometryState::GEOMETRY_FAILED};
  bool box_detected{false};
  bool pose_detected{false};
  bool pose_valid_for_grasp{false};
  geometry_msgs::msg::Pose observation_pose_base;
  cv::Rect candidate_bbox;
  int total_frame_count{0};
  int valid_frame_count{0};
  float motion_confidence{0.0F};
  float geometry_confidence{0.0F};
  double latest_sensor_timestamp_sec{0.0};
  std::string message;
};

const char * toString(MotionState state);
const char * toString(GeometryState state);
const char * toString(DetectionStatus status);

}  // namespace detect_pkg
