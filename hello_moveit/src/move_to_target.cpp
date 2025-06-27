#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_to_target",
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
    RCLCPP_ERROR(node->get_logger(), "Please use the launch file: ros2 launch hello_moveit move_to_target.launch.py");
    rclcpp::shutdown();
    return -1;
  }
  
  // 检查robot_description_semantic参数
  if (!node->has_parameter("robot_description_semantic")) {
    RCLCPP_ERROR(node->get_logger(), "robot_description_semantic parameter not found!");
    RCLCPP_ERROR(node->get_logger(), "Please use the launch file: ros2 launch hello_moveit move_to_target.launch.py");
    rclcpp::shutdown();
    return -1;
  }

  // 创建MoveGroup接口
  static const std::string PLANNING_GROUP = "arm_group";
  auto move_group = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
  
  // 等待关节状态更新
  RCLCPP_INFO(node->get_logger(), "Waiting for current robot state...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  try {
    // 获取当前位姿作为参考
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);

    // 设置运动规划参数 (为真实硬件优化)
    move_group.setMaxVelocityScalingFactor(0.1);  // 10% 最大速度 (更安全)
    move_group.setMaxAccelerationScalingFactor(0.1);  // 10% 最大加速度 
    move_group.setPlanningTime(20.0);  // 增加规划时间到20秒
    move_group.setNumPlanningAttempts(10);  // 增加规划尝试次数
    
    // 设置规划器
    move_group.setPlannerId("RRTConnectkConfigDefault");
    
    // 打印一些有用的信息
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    // 定义多个目标位置
    std::vector<geometry_msgs::msg::Pose> target_poses;
    
    // 目标位置1: 基于当前位置的相对移动（更小的移动距离）
    geometry_msgs::msg::Pose target_pose1 = current_pose.pose;
    target_pose1.position.x -= 0.01;  // 向后移动1cm (更保守)
    target_pose1.position.z += 0.01;  // 向上移动1cm (更保守)
    target_poses.push_back(target_pose1);
    
    // 目标位置2: 回到原始位置
    geometry_msgs::msg::Pose target_pose2 = current_pose.pose;
    // 保持原始位置，只是作为验证
    target_poses.push_back(target_pose2);
    
    // 首先尝试一些简单的关节空间运动
    RCLCPP_INFO(node->get_logger(), "Trying joint space movement first...");
    
    // 获取当前关节角度
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    RCLCPP_INFO(node->get_logger(), "Current joint values:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f rad", i, joint_values[i]);
    }
    
    // 定义一个简单的关节运动：只移动第一个关节
    std::vector<double> target_joint_values = joint_values;
    target_joint_values[0] += 0.3;
    target_joint_values[4] += 0.3;  // 第一个关节转动0.2弧度（约11.5度）- 大于容差的运动
    
    RCLCPP_INFO(node->get_logger(), "Moving joint 0 by 0.2 radians (11.5 degrees)...");
    move_group.setJointValueTarget(target_joint_values);
    
    // 计算运动规划
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (joint_success) {
      RCLCPP_INFO(node->get_logger(), "Joint space planning successful! Executing motion...");
      
      // 执行运动
      auto joint_result = move_group.execute(joint_plan);
      if (joint_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Joint motion executed successfully!");
        
        // 等待运动完成
        rclcpp::sleep_for(std::chrono::seconds(3));
        
        // 回到原始位置
        RCLCPP_INFO(node->get_logger(), "Returning to original position...");
        move_group.setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan return_plan;
        bool return_success = (move_group.plan(return_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (return_success) {
          auto return_result = move_group.execute(return_plan);
          if (return_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Returned to original position successfully!");
          }
        }
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute joint motion!");
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Joint space planning failed!");
      
      // 如果关节空间规划失败，尝试笛卡尔空间规划
      RCLCPP_INFO(node->get_logger(), "Trying Cartesian space movement...");
      
      // 依次移动到每个目标位置
      for (size_t i = 0; i < target_poses.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Moving to target position %zu:", i + 1);
        RCLCPP_INFO(node->get_logger(), "  Target: x=%.3f, y=%.3f, z=%.3f", 
                    target_poses[i].position.x,
                    target_poses[i].position.y,
                    target_poses[i].position.z);
        
        // 设置目标位姿
        move_group.setPoseTarget(target_poses[i]);
        
        // 计算运动规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
          RCLCPP_INFO(node->get_logger(), "Planning successful! Executing motion...");
          
          // 执行运动
          auto result = move_group.execute(my_plan);
          if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Motion executed successfully!");
          } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute motion!");
          }
        } else {
          RCLCPP_ERROR(node->get_logger(), "Planning failed for target position %zu", i + 1);
        }
        
        // 在动作之间等待 (给真实硬件更多时间)
        rclcpp::sleep_for(std::chrono::seconds(3));
      }
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", e.what());
  }

  RCLCPP_INFO(node->get_logger(), "Movement sequence completed!");

  // 停止executor并等待线程结束
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 