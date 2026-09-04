#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "upper_limb_interface/srv/detect_april_tag.hpp"

using namespace std::chrono_literals;

class DetectClientNode : public rclcpp::Node
{
public:
  DetectClientNode()
  : Node("detect_client_node"),
    service_name_(declare_parameter<std::string>("service_name", "detect"))
  {
    client_ = create_client<upper_limb_interface::srv::DetectAprilTag>(service_name_);
    RCLCPP_INFO(get_logger(), "检测测试客户端已启动，目标服务: %s", service_name_.c_str());
  }

  void run()
  {
    // 节点启动后等待3秒，给服务端和终端用户留出准备时间。
    rclcpp::sleep_for(3s);

    while (rclcpp::ok()) {
      if (!askUserConfirmation()) {
        RCLCPP_INFO(get_logger(), "用户选择不发送请求，客户端退出");
        break;
      }

      if (!waitForService()) {
        continue;
      }

      sendRequestAndPrintResponse();
    }
  }

private:
  bool askUserConfirmation() const
  {
    std::string input;
    while (rclcpp::ok()) {
      std::cout << "\n是否向检测服务发送一次请求？[y/n]: " << std::flush;
      if (!std::getline(std::cin, input)) {
        return false;
      }

      if (input == "y" || input == "Y" || input == "yes" || input == "YES") {
        return true;
      }
      if (input == "n" || input == "N" || input == "no" || input == "NO") {
        return false;
      }

      std::cout << "请输入 y 或 n。" << std::endl;
    }
    return false;
  }

  bool waitForService()
  {
    // 服务端可能还没启动，循环等待；Ctrl+C 时会安全退出。
    while (rclcpp::ok() && !client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "等待服务 %s 可用...", service_name_.c_str());
    }
    return rclcpp::ok();
  }

  void sendRequestAndPrintResponse()
  {
    auto request = std::make_shared<upper_limb_interface::srv::DetectAprilTag::Request>();
    request->capture_once = true;

    RCLCPP_INFO(get_logger(), "已发送检测请求，等待响应...");
    auto future = client_->async_send_request(request);

    const auto status = rclcpp::spin_until_future_complete(shared_from_this(), future);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "服务请求失败或被中断");
      return;
    }

    const auto response = future.get();
    const auto & pose = response->box_pose;

    std::cout << "\n========== 检测响应 ==========\n"
              << "success: " << (response->success ? "true" : "false") << "\n"
              << "message: " << response->message << "\n"
              << "pose.position:\n"
              << "  x: " << pose.position.x << "\n"
              << "  y: " << pose.position.y << "\n"
              << "  z: " << pose.position.z << "\n"
              << "pose.orientation:\n"
              << "  x: " << pose.orientation.x << "\n"
              << "  y: " << pose.orientation.y << "\n"
              << "  z: " << pose.orientation.z << "\n"
              << "  w: " << pose.orientation.w << "\n"
              << "==============================\n";
  }

  std::string service_name_;
  rclcpp::Client<upper_limb_interface::srv::DetectAprilTag>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectClientNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
