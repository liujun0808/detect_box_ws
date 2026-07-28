#include "detect_pkg/depth_box_detection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <dirent.h>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Eigenvalues>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "upper_limb_interface/srv/detect_april_tag.hpp"

namespace detect_pkg
{

const char * toString(MotionState state)
{
  switch (state) {
    case MotionState::MOVING:
      return "MOVING";
    case MotionState::STATIC_CANDIDATE:
      return "STATIC_CANDIDATE";
    case MotionState::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char * toString(GeometryState state)
{
  switch (state) {
    case GeometryState::NO_BOX:
      return "NO_BOX";
    case GeometryState::PARTIAL_ENTERING:
      return "PARTIAL_ENTERING";
    case GeometryState::PARTIAL_GEOMETRY:
      return "PARTIAL_GEOMETRY";
    case GeometryState::ESTIMATED_CENTER:
      return "ESTIMATED_CENTER";
    case GeometryState::SUFFICIENT_GEOMETRY:
      return "SUFFICIENT_GEOMETRY";
    case GeometryState::GEOMETRY_FAILED:
    default:
      return "GEOMETRY_FAILED";
  }
}

const char * toString(DetectionStatus status)
{
  switch (status) {
    case DetectionStatus::NO_BOX:
      return "NO_BOX";
    case DetectionStatus::MOVING_PARTIAL_ENTERING:
      return "MOVING_PARTIAL_ENTERING";
    case DetectionStatus::MOVING_PARTIAL_GEOMETRY:
      return "MOVING_PARTIAL_GEOMETRY";
    case DetectionStatus::MOVING_ESTIMATED_CENTER:
      return "MOVING_ESTIMATED_CENTER";
    case DetectionStatus::MOVING_SUFFICIENT_GEOMETRY:
      return "MOVING_SUFFICIENT_GEOMETRY";
    case DetectionStatus::MOTION_UNCERTAIN:
      return "MOTION_UNCERTAIN";
    case DetectionStatus::STATIC_CANDIDATE:
      return "STATIC_CANDIDATE";
    case DetectionStatus::DETECTION_FAILED:
    default:
      return "DETECTION_FAILED";
  }
}

namespace
{

float percentile(std::vector<float> values, float ratio)
{
  if (values.empty()) {
    return 0.0F;
  }
  ratio = std::clamp(ratio, 0.0F, 1.0F);
  const std::size_t index =
    static_cast<std::size_t>(std::round(ratio * static_cast<float>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

cv::Rect clampRect(const cv::Rect & rect, const cv::Size & image_size)
{
  const int x0 = std::clamp(rect.x, 0, image_size.width);
  const int y0 = std::clamp(rect.y, 0, image_size.height);
  const int x1 = std::clamp(rect.x + rect.width, 0, image_size.width);
  const int y1 = std::clamp(rect.y + rect.height, 0, image_size.height);
  return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0));
}

cv::Mat buildPairMotionMask(
  const cv::Mat & old_depth,
  const cv::Mat & new_depth,
  float depth_scale_m,
  float threshold_m)
{
  const int threshold_raw =
    std::max(1, static_cast<int>(std::round(threshold_m / depth_scale_m)));
  cv::Mat valid = (old_depth > 0) & (new_depth > 0);
  cv::Mat abs_diff;
  cv::absdiff(old_depth, new_depth, abs_diff);
  return valid & (abs_diff > threshold_raw);
}

bool makePlaneFromThreePoints(
  const Eigen::Vector3f & p0,
  const Eigen::Vector3f & p1,
  const Eigen::Vector3f & p2,
  Plane3D & plane)
{
  Eigen::Vector3f normal = (p1 - p0).cross(p2 - p0);
  const float norm = normal.norm();
  if (norm < 1e-6F) {
    return false;
  }
  normal /= norm;
  plane.normal = normal;
  plane.d = -normal.dot(p0);
  return true;
}

Plane3D refinePlane(
  const std::vector<Eigen::Vector3f> & points,
  const std::vector<std::size_t> & inliers)
{
  Plane3D plane;
  plane.inlier_indices = inliers;
  for (const std::size_t index : inliers) {
    plane.centroid += points[index];
  }
  plane.centroid /= static_cast<float>(inliers.size());

  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const std::size_t index : inliers) {
    const Eigen::Vector3f delta = points[index] - plane.centroid;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<float>(inliers.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  plane.normal = solver.eigenvectors().col(0).normalized();
  if (plane.normal.dot(plane.centroid) > 0.0F) {
    plane.normal = -plane.normal;
  }
  plane.d = -plane.normal.dot(plane.centroid);

  float squared_sum = 0.0F;
  for (const std::size_t index : inliers) {
    const float distance = plane.normal.dot(points[index]) + plane.d;
    squared_sum += distance * distance;
  }
  plane.rmse_m = std::sqrt(squared_sum / static_cast<float>(inliers.size()));
  return plane;
}

std::optional<Plane3D> fitPlaneRansac(
  const std::vector<Eigen::Vector3f> & points,
  int max_iterations,
  float inlier_distance_m,
  int min_inliers)
{
  if (points.size() < 3 || points.size() < static_cast<std::size_t>(min_inliers)) {
    return std::nullopt;
  }

  std::mt19937 rng(42U);
  std::uniform_int_distribution<std::size_t> distribution(0, points.size() - 1);
  std::vector<std::size_t> best_inliers;
  best_inliers.reserve(points.size());

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    const std::size_t i0 = distribution(rng);
    const std::size_t i1 = distribution(rng);
    const std::size_t i2 = distribution(rng);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    Plane3D candidate;
    if (!makePlaneFromThreePoints(points[i0], points[i1], points[i2], candidate)) {
      continue;
    }

    std::vector<std::size_t> inliers;
    inliers.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const float distance = std::abs(candidate.normal.dot(points[index]) + candidate.d);
      if (distance <= inlier_distance_m) {
        inliers.push_back(index);
      }
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
    }
  }

  if (best_inliers.size() < static_cast<std::size_t>(min_inliers)) {
    return std::nullopt;
  }
  return refinePlane(points, best_inliers);
}

PlaneBasis makePlaneBasis(const Plane3D & plane)
{
  PlaneBasis basis;
  basis.origin_camera = plane.centroid;
  basis.normal_camera = plane.normal.normalized();
  Eigen::Vector3f reference = Eigen::Vector3f::UnitX();
  if (std::abs(basis.normal_camera.dot(reference)) > 0.90F) {
    reference = Eigen::Vector3f::UnitY();
  }
  basis.axis_u_camera = basis.normal_camera.cross(reference).normalized();
  basis.axis_v_camera = basis.normal_camera.cross(basis.axis_u_camera).normalized();
  return basis;
}

Eigen::Vector2f projectToPlane(const Eigen::Vector3f & point_camera, const PlaneBasis & basis)
{
  const Eigen::Vector3f delta = point_camera - basis.origin_camera;
  return Eigen::Vector2f(delta.dot(basis.axis_u_camera), delta.dot(basis.axis_v_camera));
}

Eigen::Vector3f unprojectFromPlane(const Eigen::Vector2f & uv, const PlaneBasis & basis)
{
  return basis.origin_camera + uv.x() * basis.axis_u_camera + uv.y() * basis.axis_v_camera;
}

bool ensureDirectory(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  struct stat path_stat;
  if (stat(path.c_str(), &path_stat) == 0) {
    return S_ISDIR(path_stat.st_mode);
  }
  return mkdir(path.c_str(), 0755) == 0;
}

std::string makeDebugPath(const std::string & dir, const std::string & name, const char * suffix)
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::ostringstream oss;
  oss << dir << "/" << name << "_" << millis << suffix;
  return oss.str();
}

geometry_msgs::msg::Pose poseFromCameraEstimate(
  const GeometryEstimate & estimate,
  const Eigen::Matrix4d & camera_to_base)
{
  Eigen::Matrix4d camera_to_box = Eigen::Matrix4d::Identity();
  camera_to_box.block<3, 3>(0, 0) = estimate.rotation_camera_box.cast<double>();
  camera_to_box.block<3, 1>(0, 3) = estimate.position_camera.cast<double>();

  const Eigen::Matrix4d base_to_box = camera_to_base * camera_to_box;
  Eigen::Quaterniond quaternion(base_to_box.block<3, 3>(0, 0));
  quaternion.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = base_to_box(0, 3);
  pose.position.y = base_to_box(1, 3);
  pose.position.z = base_to_box(2, 3);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

}  // namespace

class D435DepthFrameProvider
{
public:
  explicit D435DepthFrameProvider(const DepthBoxDetectionConfig & config)
  : config_(config)
  {
  }

  ~D435DepthFrameProvider()
  {
    stop();
  }

  void start()
  {
    if (started_) {
      return;
    }
    rs2::config rs_config;
    if (!config_.camera_serial_no.empty()) {
      rs_config.enable_device(config_.camera_serial_no);
    }
    rs_config.enable_stream(
      RS2_STREAM_DEPTH,
      config_.depth_width,
      config_.depth_height,
      RS2_FORMAT_Z16,
      config_.depth_fps);

    profile_ = pipeline_.start(rs_config);
    const auto sensor = profile_.get_device().first<rs2::depth_sensor>();
    depth_scale_m_ = sensor.get_depth_scale();
    const auto stream_profile =
      profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics_ = stream_profile.get_intrinsics();

    for (int i = 0; i < config_.camera_warmup_frames; ++i) {
      pipeline_.wait_for_frames(static_cast<unsigned int>(config_.frame_timeout_ms));
    }
    started_ = true;
  }

  void stop()
  {
    if (!started_) {
      return;
    }
    pipeline_.stop();
    started_ = false;
  }

  bool collectWindow(std::vector<DepthFrame> & frames, std::string & message)
  {
    frames.clear();
    start();

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.capture_window_sec));

    double last_timestamp = -std::numeric_limits<double>::infinity();
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        const rs2::frameset frameset =
          pipeline_.wait_for_frames(static_cast<unsigned int>(config_.frame_timeout_ms));
        const rs2::depth_frame depth_frame = frameset.get_depth_frame();
        if (!depth_frame) {
          continue;
        }
        const double timestamp_sec = depth_frame.get_timestamp() * 0.001;
        if (timestamp_sec <= last_timestamp) {
          continue;
        }
        last_timestamp = timestamp_sec;

        cv::Mat depth_view(
          depth_frame.get_height(),
          depth_frame.get_width(),
          CV_16UC1,
          const_cast<void *>(depth_frame.get_data()),
          static_cast<std::size_t>(depth_frame.get_stride_in_bytes()));

        DepthFrame output;
        output.depth_u16 = depth_view.clone();
        output.timestamp_sec = timestamp_sec;
        output.intrinsics = intrinsics_;
        output.depth_scale_m = depth_scale_m_;
        output.sequence_index = sequence_index_++;
        frames.push_back(std::move(output));
      } catch (const rs2::error & error) {
        message = std::string("RealSense深度采集失败: ") + error.what();
        return false;
      }
    }

    if (frames.size() < 2U) {
      message = "短窗口内深度帧不足";
      return false;
    }
    message = "深度短窗口采集完成";
    return true;
  }

private:
  DepthBoxDetectionConfig config_;
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2_intrinsics intrinsics_{};
  float depth_scale_m_{0.001F};
  int sequence_index_{0};
  bool started_{false};
};

class DepthBoxDetector
{
public:
  explicit DepthBoxDetector(const DepthBoxDetectionConfig & config)
  : config_(config)
  {
  }

  DetectionResult detect(const std::vector<DepthFrame> & frames) const
  {
    DetectionResult result;
    result.pose_valid_for_grasp = false;
    result.total_frame_count = static_cast<int>(frames.size());
    result.valid_frame_count = static_cast<int>(frames.size());
    if (frames.empty()) {
      result.message = "没有可用深度帧";
      return result;
    }
    result.latest_sensor_timestamp_sec = frames.back().timestamp_sec;

    cv::Mat motion_mask;
    MotionEstimate motion;
    if (!buildMotionMask(frames, motion_mask, motion)) {
      result.status = DetectionStatus::NO_BOX;
      result.geometry_state = GeometryState::NO_BOX;
      result.message = "未从间隔帧差分中找到运动候选";
      saveDebug(frames.back(), motion_mask, result.candidate_bbox, false);
      return result;
    }
    result.motion_state = motion.state;
    result.motion_confidence = motion.confidence;

    std::optional<cv::Rect> motion_bbox = extractMotionBbox(motion_mask);
    if (!motion_bbox) {
      result.status = DetectionStatus::NO_BOX;
      result.geometry_state = GeometryState::NO_BOX;
      result.message = "运动掩膜为空";
      saveDebug(frames.back(), motion_mask, result.candidate_bbox, false);
      return result;
    }

    const cv::Size image_size = frames.back().depth_u16.size();
    result.candidate_bbox = expandCandidate(*motion_bbox, image_size);
    const auto samples = buildPointSamples(frames.back(), result.candidate_bbox);
    GeometryEstimate geometry = estimateGeometry(samples, result.candidate_bbox, image_size);

    result.box_detected = geometry.geometry_state != GeometryState::NO_BOX &&
      geometry.geometry_state != GeometryState::GEOMETRY_FAILED;
    result.pose_detected = geometry.pose_detected;
    result.geometry_state = geometry.geometry_state;
    result.geometry_confidence = geometry.confidence;
    if (geometry.pose_detected) {
      result.observation_pose_base = poseFromCameraEstimate(geometry, config_.camera_to_base);
    }

    result.status = combineStatus(result.motion_state, geometry.geometry_state);
    std::ostringstream message;
    message << "status=" << toString(result.status)
            << ", motion=" << toString(result.motion_state)
            << ", geometry=" << toString(result.geometry_state)
            << ", frames=" << result.total_frame_count
            << ", bbox=[" << result.candidate_bbox.x << ", " << result.candidate_bbox.y
            << ", " << result.candidate_bbox.width << ", " << result.candidate_bbox.height << "]"
            << ", visible_ratio=[" << std::fixed << std::setprecision(2)
            << geometry.length_visible_ratio << ", " << geometry.width_visible_ratio << "]"
            << ", pose_valid_for_grasp=false";
    result.message = message.str();
    saveDebug(frames.back(), motion_mask, result.candidate_bbox, result.pose_detected);
    return result;
  }

private:
  bool buildMotionMask(
    const std::vector<DepthFrame> & frames,
    cv::Mat & motion_mask,
    MotionEstimate & motion) const
  {
    if (frames.size() <= static_cast<std::size_t>(config_.motion_frame_gap)) {
      return false;
    }

    cv::Mat vote_count = cv::Mat::zeros(frames.front().depth_u16.size(), CV_16UC1);
    std::vector<float> bbox_centers;
    for (std::size_t i = 0; i + static_cast<std::size_t>(config_.motion_frame_gap) < frames.size(); ++i) {
      const cv::Mat pair_mask = buildPairMotionMask(
        frames[i].depth_u16,
        frames[i + static_cast<std::size_t>(config_.motion_frame_gap)].depth_u16,
        frames[i].depth_scale_m,
        config_.depth_difference_threshold_m);
      cv::Mat pair_vote;
      pair_mask.convertTo(pair_vote, CV_16UC1, 1.0 / 255.0);
      vote_count += pair_vote;

      std::optional<cv::Rect> bbox = extractMotionBbox(pair_mask);
      if (bbox) {
        bbox_centers.push_back(static_cast<float>(bbox->x + bbox->width * 0.5));
      }
    }

    motion_mask = vote_count >= config_.min_motion_votes;
    if (config_.morph_close_size > 1) {
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(config_.morph_close_size, config_.morph_close_size));
      cv::morphologyEx(motion_mask, motion_mask, cv::MORPH_CLOSE, kernel);
    }
    if (config_.morph_open_size > 1) {
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(config_.morph_open_size, config_.morph_open_size));
      cv::morphologyEx(motion_mask, motion_mask, cv::MORPH_OPEN, kernel);
    }
    if (config_.dilate_size > 1) {
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(config_.dilate_size, config_.dilate_size));
      cv::dilate(motion_mask, motion_mask, kernel);
    }

    if (bbox_centers.size() >= 2U) {
      const auto [min_it, max_it] = std::minmax_element(bbox_centers.begin(), bbox_centers.end());
      const float span_px = *max_it - *min_it;
      motion.position_span_m =
        span_px * frames.back().depth_scale_m * 2.0F;  // coarse confidence cue only
      motion.confidence = std::min(1.0F, static_cast<float>(bbox_centers.size()) / 8.0F);
      motion.state = motion.position_span_m >= config_.moving_position_span_threshold_m ?
        MotionState::MOVING : MotionState::STATIC_CANDIDATE;
    } else {
      motion.confidence = 0.3F;
      motion.state = MotionState::UNKNOWN;
    }
    return cv::countNonZero(motion_mask) >= config_.min_motion_area_px;
  }

  std::optional<cv::Rect> extractMotionBbox(const cv::Mat & mask) const
  {
    if (mask.empty()) {
      return std::nullopt;
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0.0;
    cv::Rect best_rect;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area > best_area) {
        best_area = area;
        best_rect = cv::boundingRect(contour);
      }
    }
    if (best_area < static_cast<double>(config_.min_motion_area_px)) {
      return std::nullopt;
    }
    return best_rect;
  }

  cv::Rect expandCandidate(const cv::Rect & motion_bbox, const cv::Size & image_size) const
  {
    return clampRect(
      cv::Rect(
        motion_bbox.x - config_.candidate_expand_left_px,
        motion_bbox.y - config_.candidate_expand_up_px,
        motion_bbox.width + config_.candidate_expand_left_px + config_.candidate_expand_right_px,
        motion_bbox.height + config_.candidate_expand_up_px + config_.candidate_expand_down_px),
      image_size);
  }

  std::vector<PointSample> buildPointSamples(const DepthFrame & frame, const cv::Rect & bbox) const
  {
    std::vector<PointSample> samples;
    samples.reserve(static_cast<std::size_t>(bbox.area() / std::max(1, config_.point_stride_px)));
    for (int v = bbox.y; v < bbox.y + bbox.height; v += config_.point_stride_px) {
      const auto * row = frame.depth_u16.ptr<std::uint16_t>(v);
      for (int u = bbox.x; u < bbox.x + bbox.width; u += config_.point_stride_px) {
        const std::uint16_t raw_depth = row[u];
        if (raw_depth == 0U) {
          continue;
        }
        const float z = static_cast<float>(raw_depth) * frame.depth_scale_m;
        const float x = (static_cast<float>(u) - frame.intrinsics.ppx) / frame.intrinsics.fx * z;
        const float y = (static_cast<float>(v) - frame.intrinsics.ppy) / frame.intrinsics.fy * z;
        if (x < config_.roi_x_min_m || x > config_.roi_x_max_m ||
          y < config_.roi_y_min_m || y > config_.roi_y_max_m ||
          z < config_.roi_z_min_m || z > config_.roi_z_max_m)
        {
          continue;
        }
        samples.push_back(PointSample{Eigen::Vector3f(x, y, z), cv::Point(u, v)});
      }
    }
    return samples;
  }

  GeometryEstimate estimateGeometry(
    const std::vector<PointSample> & samples,
    const cv::Rect & bbox,
    const cv::Size & image_size) const
  {
    GeometryEstimate geometry;
    geometry.touches_left_border = bbox.x <= config_.border_margin_px;
    geometry.touches_right_border =
      bbox.x + bbox.width >= image_size.width - config_.border_margin_px;

    if (samples.size() < static_cast<std::size_t>(config_.min_candidate_points)) {
      geometry.geometry_state = geometry.touches_right_border ?
        GeometryState::PARTIAL_ENTERING : GeometryState::GEOMETRY_FAILED;
      geometry.confidence = geometry.touches_right_border ? 0.35F : 0.0F;
      return geometry;
    }

    std::vector<float> z_values;
    z_values.reserve(samples.size());
    for (const auto & sample : samples) {
      z_values.push_back(sample.point_camera.z());
    }
    const float near_depth = percentile(z_values, config_.near_depth_percentile);

    std::vector<Eigen::Vector3f> rim_candidates;
    rim_candidates.reserve(samples.size());
    const int top_limit = bbox.y + static_cast<int>(std::round(
      static_cast<float>(bbox.height) * config_.rim_image_top_ratio));
    for (const auto & sample : samples) {
      if (sample.pixel.y > top_limit) {
        continue;
      }
      if (std::abs(sample.point_camera.z() - near_depth) > config_.near_depth_band_m) {
        continue;
      }
      rim_candidates.push_back(sample.point_camera);
    }
    if (rim_candidates.size() < static_cast<std::size_t>(config_.min_rim_candidate_points)) {
      geometry.geometry_state = geometry.touches_right_border ?
        GeometryState::PARTIAL_ENTERING : GeometryState::PARTIAL_GEOMETRY;
      geometry.confidence = 0.40F;
      return geometry;
    }

    const auto plane = fitPlaneRansac(
      rim_candidates,
      config_.plane_max_iterations,
      config_.plane_inlier_distance_m,
      config_.plane_min_inliers);
    if (!plane) {
      geometry.geometry_state = geometry.touches_right_border ?
        GeometryState::PARTIAL_ENTERING : GeometryState::GEOMETRY_FAILED;
      geometry.confidence = geometry.touches_right_border ? 0.35F : 0.0F;
      return geometry;
    }

    const PlaneBasis basis = makePlaneBasis(*plane);
    std::vector<Eigen::Vector2f> uv_points;
    uv_points.reserve(plane->inlier_indices.size());
    for (const std::size_t index : plane->inlier_indices) {
      uv_points.push_back(projectToPlane(rim_candidates[index], basis));
    }
    if (uv_points.size() < 3U) {
      geometry.geometry_state = GeometryState::GEOMETRY_FAILED;
      return geometry;
    }

    Eigen::Vector2f uv_centroid = Eigen::Vector2f::Zero();
    for (const auto & uv : uv_points) {
      uv_centroid += uv;
    }
    uv_centroid /= static_cast<float>(uv_points.size());

    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto & uv : uv_points) {
      const Eigen::Vector2f delta = uv - uv_centroid;
      covariance += delta * delta.transpose();
    }
    covariance /= static_cast<float>(uv_points.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    Eigen::Vector2f length_axis_uv = solver.eigenvectors().col(1).normalized();
    Eigen::Vector2f width_axis_uv(-length_axis_uv.y(), length_axis_uv.x());

    float min_l = std::numeric_limits<float>::infinity();
    float max_l = -std::numeric_limits<float>::infinity();
    float min_w = std::numeric_limits<float>::infinity();
    float max_w = -std::numeric_limits<float>::infinity();
    for (const auto & uv : uv_points) {
      const Eigen::Vector2f delta = uv - uv_centroid;
      const float l = delta.dot(length_axis_uv);
      const float w = delta.dot(width_axis_uv);
      min_l = std::min(min_l, l);
      max_l = std::max(max_l, l);
      min_w = std::min(min_w, w);
      max_w = std::max(max_w, w);
    }

    geometry.visible_length_m = std::max(0.0F, max_l - min_l);
    geometry.visible_width_m = std::max(0.0F, max_w - min_w);
    if (geometry.visible_width_m > geometry.visible_length_m) {
      std::swap(geometry.visible_width_m, geometry.visible_length_m);
      std::swap(length_axis_uv, width_axis_uv);
      std::swap(min_l, min_w);
      std::swap(max_l, max_w);
    }
    geometry.length_visible_ratio =
      std::clamp(geometry.visible_length_m / config_.box_model.outer_length_m, 0.0F, 1.2F);
    geometry.width_visible_ratio =
      std::clamp(geometry.visible_width_m / config_.box_model.outer_width_m, 0.0F, 1.2F);
    geometry.plane_rmse_m = plane->rmse_m;
    geometry.valid = true;

    const bool enough_for_observation =
      geometry.length_visible_ratio >= config_.min_pose_visible_length_ratio &&
      geometry.width_visible_ratio >= config_.min_pose_visible_width_ratio &&
      !geometry.touches_right_border;
    const bool sufficient =
      geometry.length_visible_ratio >= config_.sufficient_visible_length_ratio &&
      geometry.width_visible_ratio >= config_.sufficient_visible_width_ratio &&
      !geometry.touches_right_border;

    if (sufficient) {
      geometry.geometry_state = GeometryState::SUFFICIENT_GEOMETRY;
      geometry.pose_detected = true;
    } else if (enough_for_observation) {
      geometry.geometry_state = GeometryState::ESTIMATED_CENTER;
      geometry.pose_detected = true;
    } else if (geometry.touches_right_border) {
      geometry.geometry_state = GeometryState::PARTIAL_ENTERING;
      geometry.pose_detected = false;
    } else {
      geometry.geometry_state = GeometryState::PARTIAL_GEOMETRY;
      geometry.pose_detected = false;
    }

    const Eigen::Vector2f center_uv =
      uv_centroid + length_axis_uv * ((min_l + max_l) * 0.5F) + width_axis_uv * ((min_w + max_w) * 0.5F);
    geometry.position_camera = unprojectFromPlane(center_uv, basis);

    Eigen::Vector3f length_axis_camera =
      length_axis_uv.x() * basis.axis_u_camera + length_axis_uv.y() * basis.axis_v_camera;
    length_axis_camera.normalize();
    Eigen::Vector3f normal_camera = basis.normal_camera.normalized();
    Eigen::Vector3f width_axis_camera = normal_camera.cross(length_axis_camera).normalized();
    length_axis_camera = width_axis_camera.cross(normal_camera).normalized();
    geometry.rotation_camera_box.col(0) = length_axis_camera;
    geometry.rotation_camera_box.col(1) = width_axis_camera;
    geometry.rotation_camera_box.col(2) = normal_camera;
    geometry.confidence = std::clamp(
      0.30F + 0.35F * geometry.length_visible_ratio + 0.25F * geometry.width_visible_ratio -
      5.0F * geometry.plane_rmse_m,
      0.0F,
      1.0F);
    return geometry;
  }

  DetectionStatus combineStatus(MotionState motion_state, GeometryState geometry_state) const
  {
    if (geometry_state == GeometryState::NO_BOX || geometry_state == GeometryState::GEOMETRY_FAILED) {
      return DetectionStatus::NO_BOX;
    }
    if (motion_state == MotionState::STATIC_CANDIDATE) {
      return DetectionStatus::STATIC_CANDIDATE;
    }
    if (motion_state != MotionState::MOVING) {
      return DetectionStatus::MOTION_UNCERTAIN;
    }
    switch (geometry_state) {
      case GeometryState::PARTIAL_ENTERING:
        return DetectionStatus::MOVING_PARTIAL_ENTERING;
      case GeometryState::PARTIAL_GEOMETRY:
        return DetectionStatus::MOVING_PARTIAL_GEOMETRY;
      case GeometryState::ESTIMATED_CENTER:
        return DetectionStatus::MOVING_ESTIMATED_CENTER;
      case GeometryState::SUFFICIENT_GEOMETRY:
        return DetectionStatus::MOVING_SUFFICIENT_GEOMETRY;
      default:
        return DetectionStatus::DETECTION_FAILED;
    }
  }

  void saveDebug(
    const DepthFrame & latest_frame,
    const cv::Mat & motion_mask,
    const cv::Rect & bbox,
    bool success) const
  {
    if (!ensureDirectory(config_.debug_output_dir)) {
      return;
    }
    if (!motion_mask.empty()) {
      cv::imwrite(
        makeDebugPath(config_.debug_output_dir, success ? "motion_success" : "motion_failed", ".png"),
        motion_mask);
    }
    if (!latest_frame.depth_u16.empty()) {
      cv::Mat depth_8u;
      latest_frame.depth_u16.convertTo(depth_8u, CV_8U, 255.0 / 4000.0);
      cv::Mat depth_color;
      cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_TURBO);
      if (bbox.area() > 0) {
        cv::rectangle(depth_color, bbox, cv::Scalar(0, 255, 255), 2);
      }
      cv::imwrite(
        makeDebugPath(config_.debug_output_dir, success ? "depth_success" : "depth_failed", ".png"),
        depth_color);
    }
  }

  DepthBoxDetectionConfig config_;
};

class DepthBoxDetectServerNode : public rclcpp::Node
{
public:
  using DetectAprilTag = upper_limb_interface::srv::DetectAprilTag;

  DepthBoxDetectServerNode()
  : Node("detect_server_node")
  {
    config_ = loadConfig();
    provider_ = std::make_unique<D435DepthFrameProvider>(config_);
    detector_ = std::make_unique<DepthBoxDetector>(config_);

    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = create_service<DetectAprilTag>(
      config_.service_name,
      std::bind(&DepthBoxDetectServerNode::handleDetectRequest, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    box_pose_frame_id_ = declare_parameter<std::string>("box_pose_frame_id", "base_link");
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      declare_parameter<std::string>("box_pose_topic", "box_pose"),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    RCLCPP_INFO(
      get_logger(),
      "深度开口箱检测服务已启动: service=%s, depth=%dx%d@%d, window=%.2fs, pose_valid_for_grasp固定为false",
      config_.service_name.c_str(),
      config_.depth_width,
      config_.depth_height,
      config_.depth_fps,
      config_.capture_window_sec);
  }

private:
  DepthBoxDetectionConfig loadConfig()
  {
    DepthBoxDetectionConfig config;
    config.service_name = declare_parameter<std::string>("service_name", "detect");
    config.camera_serial_no = declare_parameter<std::string>("camera_serial_no", "");
    config.depth_width = declare_parameter<int>("depth_width", config.depth_width);
    config.depth_height = declare_parameter<int>("depth_height", config.depth_height);
    config.depth_fps = declare_parameter<int>("depth_fps", config.depth_fps);
    config.camera_warmup_frames = declare_parameter<int>("camera_warmup_frames", config.camera_warmup_frames);
    config.frame_timeout_ms = declare_parameter<int>("frame_timeout_ms", config.frame_timeout_ms);
    config.capture_window_sec = declare_parameter<double>("capture_window_sec", config.capture_window_sec);

    config.motion_frame_gap = declare_parameter<int>("motion_frame_gap", config.motion_frame_gap);
    config.min_motion_votes = declare_parameter<int>("min_motion_votes", config.min_motion_votes);
    config.depth_difference_threshold_m =
      declare_parameter<double>("depth_difference_threshold_m", config.depth_difference_threshold_m);
    config.min_motion_area_px = declare_parameter<int>("min_motion_area_px", config.min_motion_area_px);

    config.candidate_expand_left_px =
      declare_parameter<int>("candidate_expand_left_px", config.candidate_expand_left_px);
    config.candidate_expand_right_px =
      declare_parameter<int>("candidate_expand_right_px", config.candidate_expand_right_px);
    config.candidate_expand_up_px =
      declare_parameter<int>("candidate_expand_up_px", config.candidate_expand_up_px);
    config.candidate_expand_down_px =
      declare_parameter<int>("candidate_expand_down_px", config.candidate_expand_down_px);

    config.roi_x_min_m = declare_parameter<double>("roi_x_min_m", config.roi_x_min_m);
    config.roi_x_max_m = declare_parameter<double>("roi_x_max_m", config.roi_x_max_m);
    config.roi_y_min_m = declare_parameter<double>("roi_y_min_m", config.roi_y_min_m);
    config.roi_y_max_m = declare_parameter<double>("roi_y_max_m", config.roi_y_max_m);
    config.roi_z_min_m = declare_parameter<double>("roi_z_min_m", config.roi_z_min_m);
    config.roi_z_max_m = declare_parameter<double>("roi_z_max_m", config.roi_z_max_m);

    config.box_model.outer_length_m =
      declare_parameter<double>("box_outer_length_m", config.box_model.outer_length_m);
    config.box_model.outer_width_m =
      declare_parameter<double>("box_outer_width_m", config.box_model.outer_width_m);
    config.box_model.outer_height_m =
      declare_parameter<double>("box_outer_height_m", config.box_model.outer_height_m);
    config.debug_output_dir = declare_parameter<std::string>("debug_output_dir", config.debug_output_dir);

    const std::vector<double> default_camera_to_base = {
      0.0000, -0.342020, 0.939693, 0.12972,
      -1.000000, 0.000000, 0.000000, 0.03250,
      0.000000, -0.939693, -0.342020, 0.24561,
      0.000000, 0.000000, 0.000000, 1.000000};
    const auto camera_to_base_values =
      declare_parameter<std::vector<double>>("camera_to_base_row_major", default_camera_to_base);
    if (camera_to_base_values.size() != 16U) {
      throw std::runtime_error("camera_to_base_row_major必须包含16个数");
    }
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        config.camera_to_base(row, col) = camera_to_base_values[static_cast<std::size_t>(row * 4 + col)];
      }
    }

    const auto fallback_pose_values = declare_parameter<std::vector<double>>(
      "fallback_pose",
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    fallback_pose_ = makePoseFromVector(fallback_pose_values);
    return config;
  }

  void handleDetectRequest(
    const std::shared_ptr<DetectAprilTag::Request> request,
    std::shared_ptr<DetectAprilTag::Response> response)
  {
    (void)request;
    response->success = false;
    response->box_pose = fallback_pose_;

    std::vector<DepthFrame> frames;
    std::string message;
    try {
      if (!provider_->collectWindow(frames, message)) {
        response->message = message;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
      const DetectionResult result = detector_->detect(frames);
      response->success = result.pose_detected;
      response->message = result.message;
      if (result.pose_detected) {
        response->box_pose = result.observation_pose_base;
        publishMarker(result.observation_pose_base);
      }
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const rs2::error & error) {
      response->message = std::string("RealSense异常: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->message = std::string("深度检测异常: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  geometry_msgs::msg::Pose makePoseFromVector(const std::vector<double> & values) const
  {
    if (values.size() != 7U) {
      throw std::runtime_error("fallback_pose必须是7元素数组 [x, y, z, qx, qy, qz, qw]");
    }
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("fallback_pose不能包含NaN或Inf");
      }
    }
    const double norm = std::sqrt(
      values[3] * values[3] + values[4] * values[4] +
      values[5] * values[5] + values[6] * values[6]);
    if (norm < 1e-9) {
      throw std::runtime_error("fallback_pose四元数模长不能为0");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = values[0];
    pose.position.y = values[1];
    pose.position.z = values[2];
    pose.orientation.x = values[3] / norm;
    pose.orientation.y = values[4] / norm;
    pose.orientation.z = values[5] / norm;
    pose.orientation.w = values[6] / norm;
    return pose;
  }

  void publishMarker(const geometry_msgs::msg::Pose & pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = box_pose_frame_id_;
    marker.ns = "depth_detected_box_observation";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = config_.box_model.outer_length_m;
    marker.scale.y = config_.box_model.outer_width_m;
    marker.scale.z = config_.box_model.outer_height_m;
    marker.color.r = 0.1F;
    marker.color.g = 0.55F;
    marker.color.b = 1.0F;
    marker.color.a = 0.45F;
    marker_publisher_->publish(marker);
  }

  DepthBoxDetectionConfig config_;
  std::string box_pose_frame_id_;
  geometry_msgs::msg::Pose fallback_pose_;
  std::unique_ptr<D435DepthFrameProvider> provider_;
  std::unique_ptr<DepthBoxDetector> detector_;
  rclcpp::Service<DetectAprilTag>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

}  // namespace detect_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<detect_pkg::DepthBoxDetectServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
