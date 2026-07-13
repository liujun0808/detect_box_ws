#include "detect_pkg/detect.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <dirent.h>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>

namespace detect_pkg
{

DetectServerNode::DetectServerNode()
: Node("detect_server_node"),
  has_color_image_(false),
  has_camera_info_(false),
  latest_color_frame_id_(0),
  service_name_(declare_parameter<std::string>("service_name", "detect")),
  image_topic_(declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw")),
  camera_info_topic_(
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info")),
  image_qos_(declare_parameter<std::string>("image_qos", "reliable")),
  debug_image_save_prefix_(declare_parameter<std::string>("debug_image_save_prefix", "apriltag_detection")),
  debug_image_save_dir_(declare_parameter<std::string>("debug_image_save_dir", "debug_img")),
  tag_size_m_(declare_parameter<double>("tag_size_m", 0.08)),
  max_detection_attempts_(declare_parameter<int>("max_detection_attempts", 5)),
  new_frame_timeout_ms_(declare_parameter<int>("new_frame_timeout_ms", 1000))
{
  if (max_detection_attempts_ < 1) {
    RCLCPP_WARN(get_logger(), "max_detection_attempts=%d无效，已改为1", max_detection_attempts_);
    max_detection_attempts_ = 1;
  }
  if (new_frame_timeout_ms_ < 1) {
    RCLCPP_WARN(get_logger(), "new_frame_timeout_ms=%d无效，已改为1", new_frame_timeout_ms_);
    new_frame_timeout_ms_ = 1;
  }

  const auto fallback_pose_values = declare_parameter<std::vector<double>>(
    "fallback_pose",
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}); // xyz xyzw
  std::string fallback_pose_message;
  if (!makePoseFromVector(fallback_pose_values, fallback_pose_, fallback_pose_message)) {
    throw std::runtime_error("fallback_pose参数无效: " + fallback_pose_message);
  }
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&DetectServerNode::onParametersSet, this, std::placeholders::_1));

  const auto box_tag_ids = declare_parameter<std::vector<int64_t>>("box_tag_ids", {10, 24});

  const auto box_tag_positions_m = declare_parameter<std::vector<double>>(
    "box_tag_positions_m",
    {-0.1475, 0.1255, -0.0305, 0.0, 0.0, 0.00}); // 后表面 0.08
    // {-0.1005, 0.16, 0.01, -0.1005, -0.16, -0.01}); // mujoco 仿真
  const auto box_tag_rotations_row_major = declare_parameter<std::vector<double>>(
    "box_tag_rotations_row_major",
    {
      0.0, 0.0, -1.0,
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0,
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0
    });

  if (box_tag_positions_m.size() != box_tag_ids.size() * 3) {
    throw std::runtime_error("box_tag_positions_m参数数量必须等于box_tag_ids数量的3倍");
  }
  if (box_tag_rotations_row_major.size() != box_tag_ids.size() * 9) {
    throw std::runtime_error(
            "box_tag_rotations_row_major参数数量必须等于box_tag_ids数量的9倍");
  }

  for (std::size_t i = 0; i < box_tag_ids.size(); ++i) {
    const int32_t tag_id = static_cast<int32_t>(box_tag_ids[i]);
    const cv::Matx33d rotation_box_tag(
      box_tag_rotations_row_major[i * 9],
      box_tag_rotations_row_major[i * 9 + 1],
      box_tag_rotations_row_major[i * 9 + 2],
      box_tag_rotations_row_major[i * 9 + 3],
      box_tag_rotations_row_major[i * 9 + 4],
      box_tag_rotations_row_major[i * 9 + 5],
      box_tag_rotations_row_major[i * 9 + 6],
      box_tag_rotations_row_major[i * 9 + 7],
      box_tag_rotations_row_major[i * 9 + 8]);

    const cv::Matx33d orthogonality_error =
      rotation_box_tag.t() * rotation_box_tag - cv::Matx33d::eye();
    double squared_error_sum = 0.0;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        squared_error_sum += orthogonality_error(row, col) * orthogonality_error(row, col);
      }
    }
    const double determinant = cv::determinant(cv::Mat(rotation_box_tag));
    if (std::sqrt(squared_error_sum) > 1e-6 || std::abs(determinant - 1.0) > 1e-6) {
      throw std::runtime_error(
              "box_tag_rotations_row_major中存在非合法旋转矩阵（要求R^T*R=I且det(R)=1）");
    }

    tag_poses_in_box_[tag_id] = TagPoseInBox{
      cv::Vec3d(
        box_tag_positions_m[i * 3],
        box_tag_positions_m[i * 3 + 1],
        box_tag_positions_m[i * 3 + 2]),
      rotation_box_tag};
  }

  april_tag_dictionary_ =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

  detector_parameters_ = cv::aruco::DetectorParameters::create();

  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  if (image_qos_ == "best_effort" || image_qos_ == "sensor_data") {
    camera_qos = rclcpp::SensorDataQoS();
  } else if (image_qos_ != "reliable") {
    RCLCPP_WARN(
      get_logger(),
      "未知 image_qos=%s，使用 reliable；可选 reliable/best_effort/sensor_data",
      image_qos_.c_str());
  }

  camera_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions camera_subscription_options;
  camera_subscription_options.callback_group = camera_callback_group_;

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    camera_qos,
    std::bind(&DetectServerNode::imageCallback, this, std::placeholders::_1),
    camera_subscription_options);

  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    camera_qos,
    std::bind(&DetectServerNode::cameraInfoCallback, this, std::placeholders::_1),
    camera_subscription_options);

  service_ = create_service<DetectAprilTag>(
    service_name_,
    std::bind(
      &DetectServerNode::handleDetectRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_callback_group_);
  
  // 外參矩陣初始化
    // camera2base<<0.0000, -0.342020,  0.939693,  0.138680,  // 真机 高腰
    //           -1.000000,  0.000000,  0.000000,  0.03250,
    //             0.000000, -0.939693, -0.342020,  0.29242,
    //           0.000000,  0.000000,  0.000000,  1.000000;
    camera2base <<
      0.0000, -0.342020,  0.939693,  0.12972,  // 真机 低腰
      -1.000000,  0.000000,  0.000000,  0.03250,
      0.000000, -0.939693, -0.342020,  0.24561,
      0.000000,  0.000000,  0.000000,  1.000000;
    // camera2base<<0.000000, -0.500134,  0.865948,  0.130000, // mujoco 仿真
    //         -1.000000,  0.000000,  0.000000,  0.000000,
    //           0.000000, -0.865948, -0.500134,  0.250000,
    //         0.000000,  0.000000,  0.000000,  1.000000;
  RCLCPP_INFO(
    get_logger(),
    "AprilTag检测服务已启动: %s, image_topic=%s, camera_info_topic=%s, image_qos=%s, tag_size_m=%.4f, box_tag_count=%zu, debug_image_save_dir=%s",
    service_name_.c_str(),
    image_topic_.c_str(),
    camera_info_topic_.c_str(),
    image_qos_.c_str(),
    tag_size_m_,
    tag_poses_in_box_.size(),
    debug_image_save_dir_.c_str());
}

void DetectServerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->height == 0 || msg->width == 0 || msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到空的彩色图像消息");
    return;
  }

  cv::Mat color_image;
  if (msg->encoding == "bgr8") {
    cv::Mat view(
      static_cast<int>(msg->height),
      static_cast<int>(msg->width),
      CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()),
      msg->step);
    color_image = view.clone();
  } else if (msg->encoding == "rgb8") {
    cv::Mat view(
      static_cast<int>(msg->height),
      static_cast<int>(msg->width),
      CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()),
      msg->step);
    cv::cvtColor(view, color_image, cv::COLOR_RGB2BGR);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "暂不支持的图像编码: %s，仅支持bgr8/rgb8",
      msg->encoding.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(camera_data_mutex_);
  latest_color_image_ = color_image;
  has_color_image_ = true;
  ++latest_color_frame_id_;
  camera_frame_cv_.notify_all();
}

void DetectServerNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(camera_data_mutex_);
  latest_camera_info_ = *msg;
  has_camera_info_ = true;
  camera_frame_cv_.notify_all();
}

void DetectServerNode::handleDetectRequest(
  const std::shared_ptr<DetectAprilTag::Request> request,
  std::shared_ptr<DetectAprilTag::Response> response)
{
  (void)request;

  response->box_pose = fallbackPose();
  response->success = false;

  uint64_t last_used_frame_id = 0;
  {
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    last_used_frame_id = latest_color_frame_id_;
  }

  std::string last_failure_message = "尚未开始检测";
  cv::Mat last_debug_image;
  for (int attempt = 1; attempt <= max_detection_attempts_; ++attempt) {
    cv::Mat color_image;
    sensor_msgs::msg::CameraInfo camera_info;
    uint64_t captured_frame_id = last_used_frame_id;
    std::string message;

    if (!waitForNewColorFrame(
        last_used_frame_id,
        color_image,
        camera_info,
        captured_frame_id,
        message))
    {
      std::ostringstream oss;
      oss << "第" << attempt << "/" << max_detection_attempts_ << "次等待新图失败: " << message;
      last_failure_message = oss.str();
      RCLCPP_WARN(get_logger(), "%s", last_failure_message.c_str());
      continue;
    }
    last_used_frame_id = captured_frame_id;

    geometry_msgs::msg::Pose box_pose = fallbackPose();
    cv::Mat debug_image;
    if (!detectBoxPose(color_image, camera_info, box_pose, debug_image, message)) {
      std::ostringstream oss;
      oss << "第" << attempt << "/" << max_detection_attempts_ << "次识别失败: " << message;
      last_failure_message = oss.str();
      if (!debug_image.empty()) {
        last_debug_image = debug_image;
      }
      RCLCPP_WARN(get_logger(), "%s", last_failure_message.c_str());
      continue;
    }
    RCLCPP_INFO(get_logger(), "detectBoxPose已正常返回，开始检查box位置");
    RCLCPP_INFO(
      get_logger(),
      "box在base坐标系下位置: x=%.6f, y=%.6f, z=%.6f m",
      box_pose.position.x,
      box_pose.position.y,
      box_pose.position.z);

    std::string validation_message;
    if (!validateBoxPosePosition(box_pose, validation_message)) {
      std::ostringstream oss;
      oss << "第" << attempt << "/" << max_detection_attempts_ << "次position检查失败: "
          << validation_message;
      last_failure_message = oss.str();
      if (!debug_image.empty()) {
        drawDebugStatus(debug_image, last_failure_message);
      }
      if (!debug_image.empty()) {
        last_debug_image = debug_image;
      }
      RCLCPP_WARN(get_logger(), "%s", last_failure_message.c_str());
      continue;
    }
    RCLCPP_INFO(get_logger(), "box位置检查通过，开始组织服务响应");

    std::ostringstream success_message;
    success_message << "第" << attempt << "/" << max_detection_attempts_ << "次检测成功: "
                    << message;
    response->success = true;
    response->message = success_message.str();
    response->box_pose = box_pose;

    RCLCPP_INFO(get_logger(), "开始保存检测成功调试图像");
    saveDebugImage(debug_image, true);
    RCLCPP_INFO(get_logger(), "检测成功调试图像保存流程结束");
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::ostringstream final_message;
  final_message << "检测失败: 已尝试" << max_detection_attempts_
                << "次，最后一次结果: " << last_failure_message;
  response->message = final_message.str();
  response->box_pose = fallbackPose();
  saveDebugImage(last_debug_image, false);
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

bool DetectServerNode::captureColorFrame(
  cv::Mat & color_image,
  sensor_msgs::msg::CameraInfo & camera_info,
  std::string & message) const
{
  std::lock_guard<std::mutex> lock(camera_data_mutex_);
  if (!has_color_image_ || latest_color_image_.empty()) {
    message = "尚未收到相机彩色图像，请确认realsense2_camera已发布image_raw";
    return false;
  }
  if (!has_camera_info_) {
    message = "尚未收到相机内参，请确认realsense2_camera已发布camera_info";
    return false;
  }
  if (latest_camera_info_.k[0] == 0.0 || latest_camera_info_.k[4] == 0.0) {
    message = "相机内参无效，fx/fy为0";
    return false;
  }

  color_image = latest_color_image_.clone();
  camera_info = latest_camera_info_;
  return true;
}

bool DetectServerNode::waitForNewColorFrame(
  uint64_t previous_frame_id,
  cv::Mat & color_image,
  sensor_msgs::msg::CameraInfo & camera_info,
  uint64_t & captured_frame_id,
  std::string & message)
{
  std::unique_lock<std::mutex> lock(camera_data_mutex_);
  const auto timeout = std::chrono::milliseconds(new_frame_timeout_ms_);
  const bool ready = camera_frame_cv_.wait_for(
    lock,
    timeout,
    [this, previous_frame_id]() {
      return has_color_image_ &&
             !latest_color_image_.empty() &&
             latest_color_frame_id_ > previous_frame_id &&
             has_camera_info_ &&
             latest_camera_info_.k[0] != 0.0 &&
             latest_camera_info_.k[4] != 0.0;
    });

  if (!ready) {
    if (!has_color_image_ || latest_color_image_.empty() ||
      latest_color_frame_id_ <= previous_frame_id)
    {
      std::ostringstream oss;
      oss << "等待新彩色图像超时(" << new_frame_timeout_ms_
          << "ms)，未收到比frame_id=" << previous_frame_id << "更新的图像";
      message = oss.str();
      return false;
    }
    if (!has_camera_info_) {
      message = "等待新彩色图像成功，但尚未收到相机内参";
      return false;
    }
    if (latest_camera_info_.k[0] == 0.0 || latest_camera_info_.k[4] == 0.0) {
      message = "等待新彩色图像成功，但相机内参无效，fx/fy为0";
      return false;
    }

    message = "等待新彩色图像超时";
    return false;
  }

  color_image = latest_color_image_.clone();
  camera_info = latest_camera_info_;
  captured_frame_id = latest_color_frame_id_;
  return true;
}

bool DetectServerNode::detectBoxPose(
  const cv::Mat & color_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  geometry_msgs::msg::Pose & box_pose,
  cv::Mat & debug_image,
  std::string & message) const
{
  if (color_image.empty()) {
    message = "彩色图像为空，返回fallback_pose";
    return false;
  }

  try {
    debug_image = color_image.clone();

    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(
      gray_image,
      april_tag_dictionary_,
      corners,
      ids,
      detector_parameters_);

    std::ostringstream detected_ids_text;
    detected_ids_text << "detected ids=[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) {
        detected_ids_text << ", ";
      }
      detected_ids_text << ids[i];
    }
    detected_ids_text << "]";

    if (ids.empty()) {
      message = "未识别到DICT_APRILTAG_36h11码，返回fallback_pose";
      if (!debug_image.empty()) {
        drawDebugStatus(debug_image, "failed: no DICT_APRILTAG_36h11 tag");
      }
      return false;
    }

    if (!debug_image.empty()) {
      // 绘制所有被 OpenCV 识别出的 AprilTag 边框和 ID，包含不属于当前 box 的 tag。
      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
    }

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    buildCameraParameters(camera_info, camera_matrix, dist_coeffs);

    std::vector<cv::Vec3d> rotation_vectors;
    std::vector<cv::Vec3d> translation_vectors;

    cv::aruco::estimatePoseSingleMarkers(
      corners,
      static_cast<float>(tag_size_m_),
      camera_matrix,
      dist_coeffs,
      rotation_vectors,
      translation_vectors);

    if (rotation_vectors.size() != ids.size() || translation_vectors.size() != ids.size()) {
      message = "AprilTag位姿估计失败，返回fallback_pose";
      if (!debug_image.empty()) {
        drawDebugStatus(debug_image, detected_ids_text.str() + "; pose estimation failed");
      }
      return false;
    }

    if (!debug_image.empty()) {
      for (std::size_t i = 0; i < rotation_vectors.size(); ++i) {
        // 在每个 tag 上绘制估计出的局部坐标轴，轴长取 tag 边长的一半，便于观察姿态方向。
        cv::drawFrameAxes(
          debug_image,
          camera_matrix,
          dist_coeffs,
          rotation_vectors[i],
          translation_vectors[i],
          static_cast<float>(tag_size_m_ * 0.5));
      }
    }

    std::vector<cv::Matx33d> box_rotation_candidates;
    std::vector<cv::Vec3d> box_translation_candidates;
    std::vector<int32_t> matched_tag_ids;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      // ids、rotation_vectors、translation_vectors 三个数组一一对应：
      // ids[i] 是第 i 个检测到的 tag 编号；
      // rotation_vectors[i] / translation_vectors[i] 是 OpenCV 根据图像角点估计出的
      // T_cam_tag，即“tag 坐标系在相机坐标系下”的位姿。
      const int32_t tag_id = ids[i];

      // 只使用在参数 box_tag_ids / box_tag_positions_m 中登记过的 tag。
      // 例如画面中出现了其他 AprilTag，它们不属于当前 box，就直接跳过。
      const auto pose_iter = tag_poses_in_box_.find(tag_id);
      if (pose_iter == tag_poses_in_box_.end()) {
        continue;
      }

      // OpenCV 的 estimatePoseSingleMarkers 输出旋转向量 rvec。
      // Rodrigues 将 rvec 转换为 3x3 旋转矩阵 R_cam_tag，便于后续做坐标变换。
      cv::Mat rotation_matrix_cv;
      cv::Rodrigues(rotation_vectors[i], rotation_matrix_cv);
      const cv::Matx33d rotation_camera_tag(rotation_matrix_cv);
      const cv::Vec3d translation_camera_tag = translation_vectors[i];
      logTagPoseInCameraFrame(tag_id, rotation_camera_tag, translation_camera_tag);

      // T_box_tag 表示 tag 坐标系在 box 坐标系下的位姿：
      // translation_box_tag 是 tag 原点在 box 坐标系下的位置，
      // rotation_box_tag 将 tag 坐标系中的向量转换到 box 坐标系。
      const cv::Vec3d translation_box_tag = pose_iter->second.translation;
      const cv::Matx33d rotation_box_tag = pose_iter->second.rotation;

      // 坐标关系：
      // T_cam_tag = T_cam_box * T_box_tag
      // 因此：
      // T_cam_box = T_cam_tag * inverse(T_box_tag)
      //
      // 展开为：
      // R_cam_box = R_cam_tag * R_box_tag^T
      // t_cam_box = t_cam_tag - R_cam_box * t_box_tag
      //
      // OpenCV测到的是 tag 原点在相机下的位置。R_cam_box * t_box_tag 是
      // "box原点到tag原点"的偏移量、但已经用相机坐标轴表示；从 t_cam_tag
      // 中减去它，才能得到 box 原点在相机下的位置。
      const cv::Matx33d rotation_camera_box =
        rotation_camera_tag * rotation_box_tag.t();
      const cv::Vec3d translation_camera_box =
        translation_camera_tag - rotation_camera_box * translation_box_tag;

      RCLCPP_INFO(get_logger(), "tag_id=%d 已完成box候选位姿计算", tag_id);

      // 每个匹配到的 tag 都能独立推算一次 box 位姿。
      // 如果两个 tag 都识别到了，后面 fuseBoxPoseCandidates 会把这些候选位姿融合。
      box_rotation_candidates.push_back(rotation_camera_box);
      box_translation_candidates.push_back(translation_camera_box);
      matched_tag_ids.push_back(tag_id);
      RCLCPP_INFO(get_logger(), "tag_id=%d 已写入box候选列表", tag_id);
    }

    if (box_rotation_candidates.empty()) {
      message = "识别到AprilTag，但未匹配到用于定位box的tag id，返回fallback_pose";
      if (!debug_image.empty()) {
        drawDebugStatus(debug_image, detected_ids_text.str() + "; no configured box tag matched");
      }
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "开始融合%zu个box位姿候选", box_rotation_candidates.size());
    box_pose = fuseBoxPoseCandidates(box_rotation_candidates, box_translation_candidates);
    RCLCPP_INFO(get_logger(), "box位姿融合及base坐标变换完成");

    std::ostringstream oss;
    oss << "使用" << matched_tag_ids.size() << "个AprilTag定位box, ids=[";
    std::ostringstream matched_ids_text;
    matched_ids_text << "matched box ids=[";
    for (std::size_t i = 0; i < matched_tag_ids.size(); ++i) {
      if (i > 0) {
        oss << ", ";
        matched_ids_text << ", ";
      }
      oss << matched_tag_ids[i];
      matched_ids_text << matched_tag_ids[i];
    }
    matched_ids_text << "]";
    oss << "], box_t=["
        << box_pose.position.x << ", "
        << box_pose.position.y << ", "
        << box_pose.position.z << "];"
        <<"box_ori=["
        <<box_pose.orientation.x<<box_pose.orientation.y<<box_pose.orientation.z<<box_pose.orientation.w<< "];";
    message = oss.str();
    if (!debug_image.empty()) {
      std::ostringstream status_text;
      status_text << detected_ids_text.str() << "; " << matched_ids_text.str()
                  << "; box_t=[" << std::fixed << std::setprecision(3)
                  << box_pose.position.x << ", "
                  << box_pose.position.y << ", "
                  << box_pose.position.z << "]";
      drawDebugStatus(debug_image, status_text.str());
    }
    return true;
  } catch (const cv::Exception & error) {
    message = std::string("OpenCV检测异常: ") + error.what() + "，返回fallback_pose";
    if (!debug_image.empty()) {
      drawDebugStatus(debug_image, message);
    }
    return false;
  } catch (const std::exception & error) {
    message = std::string("AprilTag检测异常: ") + error.what() + "，返回fallback_pose";
    if (!debug_image.empty()) {
      drawDebugStatus(debug_image, message);
    }
    return false;
  }
}

void DetectServerNode::buildCameraParameters(
  const sensor_msgs::msg::CameraInfo & camera_info,
  cv::Mat & camera_matrix,
  cv::Mat & dist_coeffs) const
{
  camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], camera_info.k[1], camera_info.k[2],
    camera_info.k[3], camera_info.k[4], camera_info.k[5],
    camera_info.k[6], camera_info.k[7], camera_info.k[8]);

  if (camera_info.d.empty()) {
    dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    return;
  }

  dist_coeffs = cv::Mat::zeros(1, static_cast<int>(camera_info.d.size()), CV_64F);
  for (std::size_t i = 0; i < camera_info.d.size(); ++i) {
    dist_coeffs.at<double>(0, static_cast<int>(i)) = camera_info.d[i];
  }
}

void DetectServerNode::logTagPoseInCameraFrame(
  int32_t tag_id,
  const cv::Matx33d & rotation_camera_tag,
  const cv::Vec3d & translation_camera_tag) const
{
  Eigen::Matrix3d eigen_rotation;
  eigen_rotation <<
    rotation_camera_tag(0, 0), rotation_camera_tag(0, 1), rotation_camera_tag(0, 2),
    rotation_camera_tag(1, 0), rotation_camera_tag(1, 1), rotation_camera_tag(1, 2),
    rotation_camera_tag(2, 0), rotation_camera_tag(2, 1), rotation_camera_tag(2, 2);

  Eigen::Quaterniond quaternion(eigen_rotation);
  quaternion.normalize();

  RCLCPP_INFO(
    get_logger(),
    "检测到 tag_id=%d 在相机坐标系下位姿: "
    "position[x=%.4f, y=%.4f, z=%.4f] m, "
    "orientation[x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
    tag_id,
    translation_camera_tag[0],
    translation_camera_tag[1],
    translation_camera_tag[2],
    quaternion.x(),
    quaternion.y(),
    quaternion.z(),
    quaternion.w());
}

geometry_msgs::msg::Pose DetectServerNode::fuseBoxPoseCandidates(
  const std::vector<cv::Matx33d> & rotation_candidates,
  const std::vector<cv::Vec3d> & translation_candidates) const
{
  if (rotation_candidates.empty() ||
    rotation_candidates.size() != translation_candidates.size())
  {
    throw std::invalid_argument("box位姿候选数量无效");
  }

  // 单个 tag 的旋转矩阵已经是合法旋转，无需再进入 OpenCV SVD。
  if (rotation_candidates.size() == 1) {
    return buildPoseMessage(rotation_candidates.front(), translation_candidates.front());
  }

  // rotation_candidates / translation_candidates 中的每一项，都是由一个 tag
  // 单独反推出的 T_cam_box。理论上，如果相机内参、tag尺寸、tag安装位置都完全准确，
  // 两个 tag 推出的 box 位姿应该一致；实际中会因为检测噪声、打印误差、贴纸误差产生偏差。

  // 先分别累加旋转矩阵和平移向量。平移是欧氏空间向量，可以直接求算术平均。
  // 旋转矩阵不能简单相加后直接使用，因为平均后的矩阵通常不再是严格正交矩阵。
  cv::Matx33d rotation_sum = cv::Matx33d::zeros();
  cv::Vec3d translation_sum(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < rotation_candidates.size(); ++i) {
    rotation_sum += rotation_candidates[i];
    translation_sum += translation_candidates[i];
  }

  // 得到旋转矩阵元素级平均值。这个 rotation_average 只是一种中间量，
  // 它接近真实旋转，但可能存在列向量不垂直、长度不为1的问题。
  const double candidate_count = static_cast<double>(rotation_candidates.size());
  cv::Mat rotation_average(rotation_sum * (1.0 / candidate_count));

  // 使用 SVD 将“近似旋转矩阵”投影回最近的合法旋转矩阵 SO(3)。
  // 对矩阵 M 做 M = U * S * Vt，最近的正交旋转矩阵可取 R = U * Vt。
  cv::SVD svd(rotation_average);
  cv::Mat rotation_orthonormal = svd.u * svd.vt;

  // 合法旋转矩阵的行列式应为 +1。如果出现 -1，说明投影成了镜像变换，
  // 需要翻转 U 的最后一列后重新计算，保证最终结果仍是右手系旋转。
  if (cv::determinant(rotation_orthonormal) < 0.0) {
    cv::Mat corrected_u = svd.u.clone();
    corrected_u.col(2) *= -1.0;
    rotation_orthonormal = corrected_u * svd.vt;
  }

  // 平移直接取平均；最后把融合后的旋转矩阵和平移向量转换为 geometry_msgs/Pose。
  const cv::Vec3d translation_average = translation_sum * (1.0 / candidate_count);
  return buildPoseMessage(cv::Matx33d(rotation_orthonormal), translation_average);
}

geometry_msgs::msg::Pose DetectServerNode::buildPoseMessage(
  const cv::Matx33d & rotation_matrix,
  const cv::Vec3d & translation_vector) const
{
  geometry_msgs::msg::Pose pose;

  Eigen::Matrix3d eigen_rotation;
  eigen_rotation <<
    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2);

  Eigen::Matrix4d box2camera = Eigen::Matrix4d::Identity();
  box2camera.block<3, 3>(0, 0) = eigen_rotation;
  box2camera(0, 3) = translation_vector[0];
  box2camera(1, 3) = translation_vector[1];
  box2camera(2, 3) = translation_vector[2];

  // T_base_box = T_base_camera * T_camera_box。
  const Eigen::Matrix4d box2base = camera2base * box2camera;
  const Eigen::Matrix3d rotation_base_box = box2base.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(rotation_base_box);
  quaternion.normalize();

  pose.position.x = box2base(0, 3);
  pose.position.y = box2base(1, 3);
  pose.position.z = box2base(2, 3);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return pose;
}

bool DetectServerNode::validateBoxPosePosition(
  const geometry_msgs::msg::Pose & box_pose,
  std::string & message) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  bool has_error = false;

  const auto append_axis_error =
    [&oss, &has_error](const char * axis, double value, double min_value, double max_value) {
      if (value >= min_value && value <= max_value) {
        return;
      }
      if (has_error) {
        oss << "; ";
      }
      oss << axis << "=" << value << "m超出范围[" << min_value << ", " << max_value << "]m";
      has_error = true;
    };

  append_axis_error("x", box_pose.position.x, 0.0, 3.0);
  append_axis_error("y", box_pose.position.y, -0.5, 0.5);
  append_axis_error("z", box_pose.position.z, -0.5, 0.3);

  if (!has_error) {
    message = "position检查通过";
    return true;
  }

  message = oss.str();
  return false;
}

void DetectServerNode::saveDebugImage(const cv::Mat & debug_image, bool success)
{
  if (debug_image.empty()) {
    return;
  }

  try {
    if (!ensureDebugImageSaveDir()) {
      return;
    }

    const std::string file_name = buildDebugImagePath(success);

    if (cv::imwrite(file_name, debug_image)) {
      RCLCPP_INFO(get_logger(), "已保存AprilTag调试图像: %s", file_name.c_str());
      pruneDebugImages(10);
    } else {
      RCLCPP_WARN(get_logger(), "保存AprilTag调试图像失败: %s", file_name.c_str());
    }
  } catch (const cv::Exception & error) {
    RCLCPP_WARN(get_logger(), "保存AprilTag调试图像失败: %s", error.what());
  }
}

bool DetectServerNode::ensureDebugImageSaveDir() const
{
  if (debug_image_save_dir_.empty()) {
    RCLCPP_WARN(get_logger(), "debug_image_save_dir为空，无法保存AprilTag调试图像");
    return false;
  }

  std::string normalized_dir = debug_image_save_dir_;
  while (normalized_dir.size() > 1 && normalized_dir.back() == '/') {
    normalized_dir.pop_back();
  }

  std::size_t search_pos = normalized_dir[0] == '/' ? 1 : 0;
  while (true) {
    const std::size_t slash_pos = normalized_dir.find('/', search_pos);
    const std::string partial_dir =
      slash_pos == std::string::npos ? normalized_dir : normalized_dir.substr(0, slash_pos);

    if (!partial_dir.empty()) {
      struct stat path_stat;
      if (stat(partial_dir.c_str(), &path_stat) != 0) {
        if (mkdir(partial_dir.c_str(), 0755) != 0 && errno != EEXIST) {
          RCLCPP_WARN(
            get_logger(),
            "创建AprilTag调试图像目录失败: %s",
            partial_dir.c_str());
          return false;
        }
      } else if (!S_ISDIR(path_stat.st_mode)) {
        RCLCPP_WARN(
          get_logger(),
          "AprilTag调试图像保存路径不是目录: %s",
          partial_dir.c_str());
        return false;
      }
    }

    if (slash_pos == std::string::npos) {
      break;
    }
    search_pos = slash_pos + 1;
  }

  return true;
}

std::string DetectServerNode::buildDebugImagePath(bool success) const
{
  std::string normalized_dir = debug_image_save_dir_;
  while (normalized_dir.size() > 1 && normalized_dir.back() == '/') {
    normalized_dir.pop_back();
  }

  const auto now_time = std::chrono::system_clock::now();
  const std::time_t now_seconds = std::chrono::system_clock::to_time_t(now_time);
  std::tm local_time{};
  localtime_r(&now_seconds, &local_time);
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now_time.time_since_epoch()).count() % 1000000000LL;

  std::ostringstream file_name_stream;
  file_name_stream << std::put_time(&local_time, "%Y%m%d_%H%M%S")
                   << "_" << std::setw(9) << std::setfill('0') << nanoseconds
                   << (success ? "_success.png" : "_failed.png");
  const std::string file_name = file_name_stream.str();

  if (normalized_dir.empty() || normalized_dir == ".") {
    return file_name;
  }
  return normalized_dir + "/" + file_name;
}

void DetectServerNode::pruneDebugImages(std::size_t max_image_count) const
{
  std::string normalized_dir = debug_image_save_dir_;
  while (normalized_dir.size() > 1 && normalized_dir.back() == '/') {
    normalized_dir.pop_back();
  }
  if (normalized_dir.empty() || normalized_dir == ".") {
    normalized_dir = ".";
  }

  DIR * dir = opendir(normalized_dir.c_str());
  if (dir == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "打开AprilTag调试图像目录失败，无法清理旧图像: %s",
      normalized_dir.c_str());
    return;
  }

  struct DebugImageFile
  {
    std::string path;
    std::time_t modified_time;
  };
  std::vector<DebugImageFile> image_files;

  while (dirent * entry = readdir(dir)) {
    const std::string file_name(entry->d_name);
    if (file_name == "." || file_name == "..") {
      continue;
    }
    if (file_name.size() < 4 || file_name.substr(file_name.size() - 4) != ".png") {
      continue;
    }

    const std::string file_path =
      normalized_dir == "." ? file_name : normalized_dir + "/" + file_name;
    struct stat file_stat;
    if (stat(file_path.c_str(), &file_stat) != 0 || !S_ISREG(file_stat.st_mode)) {
      continue;
    }
    image_files.push_back(DebugImageFile{file_path, file_stat.st_mtime});
  }
  closedir(dir);

  if (image_files.size() <= max_image_count) {
    return;
  }

  std::sort(
    image_files.begin(),
    image_files.end(),
    [](const DebugImageFile & lhs, const DebugImageFile & rhs) {
      if (lhs.modified_time != rhs.modified_time) {
        return lhs.modified_time > rhs.modified_time;
      }
      return lhs.path > rhs.path;
    });

  for (std::size_t i = max_image_count; i < image_files.size(); ++i) {
    if (std::remove(image_files[i].path.c_str()) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "删除旧AprilTag调试图像失败: %s",
        image_files[i].path.c_str());
    }
  }
}

void DetectServerNode::drawDebugStatus(cv::Mat & debug_image, const std::string & text) const
{
  // 当前 ARM64/OpenCV 运行环境在检测完成后进入 putText 时发生段错误。
  // 状态文本不参与检测结果；保留边框、坐标轴和原调试图保存，暂不叠加文字。
  (void)debug_image;
  (void)text;
}

geometry_msgs::msg::Pose DetectServerNode::fallbackPose() const
{
  std::lock_guard<std::mutex> lock(fallback_pose_mutex_);
  return fallback_pose_;
}

bool DetectServerNode::makePoseFromVector(
  const std::vector<double> & values,
  geometry_msgs::msg::Pose & pose,
  std::string & message) const
{
  if (values.size() != 7) {
    std::ostringstream oss;
    oss << "需要7个数 [x, y, z, qx, qy, qz, qw]，实际为" << values.size();
    message = oss.str();
    return false;
  }

  for (double value : values) {
    if (!std::isfinite(value)) {
      message = "不能包含NaN或Inf";
      return false;
    }
  }

  const double qx = values[3];
  const double qy = values[4];
  const double qz = values[5];
  const double qw = values[6];
  const double quaternion_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (quaternion_norm < 1e-9) {
    message = "四元数模长不能为0";
    return false;
  }

  pose.position.x = values[0];
  pose.position.y = values[1];
  pose.position.z = values[2];
  pose.orientation.x = qx / quaternion_norm;
  pose.orientation.y = qy / quaternion_norm;
  pose.orientation.z = qz / quaternion_norm;
  pose.orientation.w = qw / quaternion_norm;
  message = "fallback_pose参数有效";
  return true;
}

rcl_interfaces::msg::SetParametersResult DetectServerNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool has_fallback_pose_update = false;
  geometry_msgs::msg::Pose updated_fallback_pose;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() != "fallback_pose") {
      continue;
    }

    std::vector<double> values;
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      values = parameter.as_double_array();
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto integer_values = parameter.as_integer_array();
      values.reserve(integer_values.size());
      for (const auto integer_value : integer_values) {
        values.push_back(static_cast<double>(integer_value));
      }
    } else {
      result.successful = false;
      result.reason = "fallback_pose必须是7元素数组 [x, y, z, qx, qy, qz, qw]";
      return result;
    }

    std::string message;
    if (!makePoseFromVector(values, updated_fallback_pose, message)) {
      result.successful = false;
      result.reason = "fallback_pose参数无效: " + message;
      return result;
    }
    has_fallback_pose_update = true;
  }

  if (has_fallback_pose_update) {
    std::lock_guard<std::mutex> lock(fallback_pose_mutex_);
    fallback_pose_ = updated_fallback_pose;
  }

  return result;
}

}  // namespace detect_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<detect_pkg::DetectServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
