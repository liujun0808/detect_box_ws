#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "upper_limb_interface/srv/detect_april_tag.hpp"

namespace detect_pkg
{

class DetectServerNode : public rclcpp::Node
{
public:
  DetectServerNode();
  ~DetectServerNode() override = default;

private:
  using DetectAprilTag = upper_limb_interface::srv::DetectAprilTag;

  void handleDetectRequest(
    const std::shared_ptr<DetectAprilTag::Request> request,
    std::shared_ptr<DetectAprilTag::Response> response);

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  bool captureColorFrame(
    cv::Mat & color_image,
    sensor_msgs::msg::CameraInfo & camera_info,
    std::string & message) const;

  bool detectTag24Pose(
    const cv::Mat & color_image,
    const sensor_msgs::msg::CameraInfo & camera_info,
    geometry_msgs::msg::Pose & tag_pose_in_camera,
    geometry_msgs::msg::Pose & tag_pose_in_base,
    cv::Mat & debug_image,
    std::string & message) const;

  void buildCameraParameters(
    const sensor_msgs::msg::CameraInfo & camera_info,
    cv::Mat & camera_matrix,
    cv::Mat & dist_coeffs) const;

  geometry_msgs::msg::Pose buildPoseMessage(
    const cv::Matx33d & rotation_matrix,
    const cv::Vec3d & translation_vector) const;

  geometry_msgs::msg::Pose transformCameraPoseToBase(
    const cv::Matx33d & rotation_camera_tag,
    const cv::Vec3d & translation_camera_tag) const;

  void showAndSaveDebugImage(const cv::Mat & debug_image, bool success);
  void drawDebugStatus(cv::Mat & debug_image, const std::string & text) const;
  geometry_msgs::msg::Pose identityPose() const;

  rclcpp::Service<DetectAprilTag>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;

  mutable std::mutex camera_data_mutex_;
  cv::Mat latest_color_image_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool has_color_image_;
  bool has_camera_info_;

  cv::Ptr<cv::aruco::Dictionary> april_tag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

  std::string service_name_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_qos_;
  std::string debug_window_name_;
  std::string debug_image_save_prefix_;
  double tag_size_m_;
  bool enable_debug_image_;
  Eigen::Matrix4d camera2base_;
};

}  // namespace detect_pkg
