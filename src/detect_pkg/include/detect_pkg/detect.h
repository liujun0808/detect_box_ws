#pragma once

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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

  struct TagPoseInBox
  {
    // T_box_tag：tag 原点的位置及 tag 坐标系相对 box 坐标系的旋转。
    cv::Vec3d translation;
    cv::Matx33d rotation;
  };

  // 服务回调：收到客户端请求后最多等待5张新图，并逐次执行 AprilTag 检测和位姿估计。
  void handleDetectRequest(
    const std::shared_ptr<DetectAprilTag::Request> request,
    std::shared_ptr<DetectAprilTag::Response> response);

  // 缓存相机节点发布的彩色图像，服务请求到来时直接使用最近一帧。
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // 缓存相机节点发布的彩色相机内参。
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // 从订阅缓存中取出最近一帧彩色图像和相机内参。
  bool captureColorFrame(
    cv::Mat & color_image,
    sensor_msgs::msg::CameraInfo & camera_info,
    std::string & message) const;

  // 等待一张比 previous_frame_id 更新的彩色图，并同时取出当前相机内参。
  bool waitForNewColorFrame(
    uint64_t previous_frame_id,
    cv::Mat & color_image,
    sensor_msgs::msg::CameraInfo & camera_info,
    uint64_t & captured_frame_id,
    std::string & message);

  // 在彩色图像中识别两个 AprilTag，并融合估计 box 坐标系在相机坐标系下的位姿。
  bool detectBoxPose(
    const cv::Mat & color_image,
    const sensor_msgs::msg::CameraInfo & camera_info,
    geometry_msgs::msg::Pose & box_pose,
    cv::Mat & debug_image,
    std::string & message) const;

  // 根据 ROS CameraInfo 构造 OpenCV 相机矩阵和畸变参数。
  void buildCameraParameters(
    const sensor_msgs::msg::CameraInfo & camera_info,
    cv::Mat & camera_matrix,
    cv::Mat & dist_coeffs) const;

  // 打印单个 AprilTag 在相机坐标系下的位姿 T_cam_tag。
  void logTagPoseInCameraFrame(
    int32_t tag_id,
    const cv::Matx33d & rotation_camera_tag,
    const cv::Vec3d & translation_camera_tag) const;

  // 将多个 tag 独立推算出的 box 位姿进行融合，得到一个统一的 box 位姿。
  geometry_msgs::msg::Pose fuseBoxPoseCandidates(
    const std::vector<cv::Matx33d> & rotation_candidates,
    const std::vector<cv::Vec3d> & translation_candidates) const;

  // 将旋转矩阵和平移向量转换为 geometry_msgs/Pose。
  geometry_msgs::msg::Pose buildPoseMessage(
    const cv::Matx33d & rotation_matrix,
    const cv::Vec3d & translation_vector) const;

  // 检查 box 在 base_link 坐标系下的位置是否落在允许范围内。
  bool validateBoxPosePosition(
    const geometry_msgs::msg::Pose & box_pose,
    std::string & message) const;

  // 保存带识别结果的图像。
  void saveDebugImage(const cv::Mat & debug_image, bool success);

  // 确保调试图像保存目录存在。
  bool ensureDebugImageSaveDir() const;

  // 拼接调试图像保存路径。
  std::string buildDebugImagePath(bool success) const;

  // 只保留调试图像目录中最新的 max_image_count 张检测图。
  void pruneDebugImages(std::size_t max_image_count) const;

  // 在图像左上角绘制本次检测的状态文字。
  void drawDebugStatus(cv::Mat & debug_image, const std::string & text) const;

  // 生成失败时返回给客户端的可配置位姿。
  geometry_msgs::msg::Pose fallbackPose() const;

  // 从 [x, y, z, qx, qy, qz, qw] 参数生成 Pose。
  bool makePoseFromVector(
    const std::vector<double> & values,
    geometry_msgs::msg::Pose & pose,
    std::string & message) const;

  // 校验并应用运行时 fallback_pose 参数更新。
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Service<DetectAprilTag>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr camera_callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // 订阅缓存：服务回调只读取最近一帧，避免服务端直接依赖 RealSense SDK。
  mutable std::mutex camera_data_mutex_;
  std::condition_variable camera_frame_cv_;
  cv::Mat latest_color_image_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool has_color_image_;
  bool has_camera_info_;
  uint64_t latest_color_frame_id_;

  // AprilTag 检测器参数和字典，默认识别 DICT_APRILTAG_36h11。
  // 注意：这里使用 opencv2/aruco.hpp 的旧接口，不使用 objdetect/ArucoDetector。
  cv::Ptr<cv::aruco::Dictionary> april_tag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

  std::string service_name_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_qos_;
  std::string debug_image_save_prefix_;
  std::string debug_image_save_dir_;
  double tag_size_m_;
  int max_detection_attempts_;
  int new_frame_timeout_ms_;
  std::map<int32_t, TagPoseInBox> tag_poses_in_box_;
  Eigen::Matrix4d camera2base;
  mutable std::mutex fallback_pose_mutex_;
  geometry_msgs::msg::Pose fallback_pose_;
};

}  // namespace detect_pkg
