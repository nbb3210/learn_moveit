#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 创建一个执行器来处理ROS回调
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // 在后台线程中启动executor
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // 等待move_group节点可用
  RCLCPP_INFO(node->get_logger(), "Waiting for move_group node...");
  while (rclcpp::ok()) {
    auto node_names = node->get_node_names();
    bool found = false;
    for (const auto& name : node_names) {
      if (name == "/move_group") {
        found = true;
        break;
      }
    }
    if (found) break;
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  
  // 检查必要的参数
  RCLCPP_INFO(node->get_logger(), "Checking for required parameters...");
  
  // 检查robot_description参数
  if (!node->has_parameter("robot_description")) {
    RCLCPP_ERROR(node->get_logger(), "robot_description parameter not found!");
    RCLCPP_ERROR(node->get_logger(), "Please use the launch file: ros2 launch hello_moveit hello_moveit.launch.py");
    rclcpp::shutdown();
    return -1;
  }
  
  // 检查robot_description_semantic参数
  if (!node->has_parameter("robot_description_semantic")) {
    RCLCPP_ERROR(node->get_logger(), "robot_description_semantic parameter not found!");
    RCLCPP_ERROR(node->get_logger(), "Please use the launch file: ros2 launch hello_moveit hello_moveit.launch.py");
    rclcpp::shutdown();
    return -1;
  }
  
  // 创建MoveGroup接口
  static const std::string PLANNING_GROUP = "arm_group";
  auto move_group = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
  
  // 等待关节状态更新
  RCLCPP_INFO(node->get_logger(), "Waiting for current robot state...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // 尝试获取当前位姿
  try {
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
    
    RCLCPP_INFO(node->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);

    // 获取当前关节角度
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    RCLCPP_INFO(node->get_logger(), "Current joint values:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f rad", i, joint_values[i]);
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current pose: %s", e.what());
  }

  // 停止executor并等待线程结束
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}