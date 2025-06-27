#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "simple_move",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 创建一个执行器来处理ROS回调
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // 在后台线程中启动executor
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // 创建MoveGroup接口
  static const std::string PLANNING_GROUP = "arm_group";
  auto move_group = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
  
  // 等待关节状态更新
  RCLCPP_INFO(node->get_logger(), "等待机器人状态更新...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  try {
    // 设置运动规划参数
    move_group.setMaxVelocityScalingFactor(0.5);  // 50% 最大速度
    move_group.setMaxAccelerationScalingFactor(0.5);  // 50% 最大加速度
    move_group.setPlanningTime(5.0);  // 规划时间限制
    
    // 方法1: 使用关节角度目标
    RCLCPP_INFO(node->get_logger(), "=== 方法1: 关节空间运动 ===");
    std::vector<double> joint_group_positions;
    joint_group_positions = move_group.getCurrentJointValues();
    
    // 修改关节角度 (适配A150 C06B机械臂的关节限制)
    if (joint_group_positions.size() >= 5) {
      // 安全的关节角度变化，符合机械臂限制
      joint_group_positions[0] = std::max(-1.57, std::min(1.57, joint_group_positions[0] + 0.3));  // joint_1
      joint_group_positions[1] = std::max(-1.57, std::min(1.57, joint_group_positions[1] - 0.2));  // joint_2  
      joint_group_positions[2] = std::max(-1.57, std::min(1.57, joint_group_positions[2] + 0.15)); // joint_3
      joint_group_positions[3] = std::max(-0.45, std::min(1.57, joint_group_positions[3] + 0.1));  // joint_4 (特殊限制)
      joint_group_positions[4] = std::max(-1.57, std::min(1.57, joint_group_positions[4] + 0.1));  // joint_5
    }
    
    move_group.setJointValueTarget(joint_group_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    bool success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(node->get_logger(), "关节空间规划成功！正在执行...");
      auto result = move_group.execute(joint_plan);
      if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "关节空间运动执行成功！");
      } else {
        RCLCPP_ERROR(node->get_logger(), "关节空间运动执行失败！");
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "关节空间规划失败！");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 方法2: 使用末端执行器位姿目标
    RCLCPP_INFO(node->get_logger(), "=== 方法2: 笛卡尔空间运动 ===");
    
    // 获取当前位姿
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
    
    // 创建目标位姿 (更保守的移动距离)
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    target_pose.position.z += 0.05;  // 向上移动5cm
    target_pose.position.x += 0.02; // 向前移动2cm
    
    RCLCPP_INFO(node->get_logger(), "目标位置: x=%.3f, y=%.3f, z=%.3f", 
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);
    
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    bool pose_success = (move_group.plan(pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (pose_success) {
      RCLCPP_INFO(node->get_logger(), "位姿规划成功！正在执行...");
      auto result = move_group.execute(pose_plan);
      if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "位姿运动执行成功！");
      } else {
        RCLCPP_ERROR(node->get_logger(), "位姿运动执行失败！");
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "位姿规划失败！");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 方法3: 回到命名位置 (如果有定义的话)
    RCLCPP_INFO(node->get_logger(), "=== 方法3: 回到预定义位置 ===");
    
    // 尝试回到 home 位置
    std::vector<std::string> named_targets = {"home", "fold", "start"};
    bool found_target = false;
    
    for (const auto& target : named_targets) {
      try {
        move_group.setNamedTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan named_plan;
        bool named_success = (move_group.plan(named_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (named_success) {
          RCLCPP_INFO(node->get_logger(), "找到命名目标 '%s'，正在移动...", target.c_str());
          auto result = move_group.execute(named_plan);
          if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "成功移动到 '%s' 位置！", target.c_str());
          } else {
            RCLCPP_ERROR(node->get_logger(), "移动到 '%s' 位置失败！", target.c_str());
          }
          found_target = true;
          break;
        }
      } catch (const std::exception& e) {
        RCLCPP_DEBUG(node->get_logger(), "命名目标 '%s' 不可用: %s", target.c_str(), e.what());
      }
    }
    
    if (!found_target) {
      RCLCPP_WARN(node->get_logger(), "未找到可用的命名目标位置");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "发生异常: %s", e.what());
  }

  RCLCPP_INFO(node->get_logger(), "运动演示完成！");

  // 停止executor并等待线程结束
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 