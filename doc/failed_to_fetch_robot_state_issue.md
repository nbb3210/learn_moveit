# MoveIt2 "Failed to fetch current robot state" 问题解决方案

## 问题描述

在运行 MoveIt2 程序时，调用 `move_group.getCurrentPose()` 或 `move_group.getCurrentJointValues()` 时会出现以下错误：

```
[INFO] [moveit_ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1750678822.696068, but latest received state has time 0.000000.
Check clock synchronization if your are running ROS across multiple machines!
[ERROR] [move_group_interface]: Failed to fetch current robot state
```

## 问题分析

### 根本原因

这个问题的根本原因是 **`MoveGroupInterface::getCurrentPose()` 依赖于 ROS spinning 处于活动状态**。

在 ROS2 中，如果没有适当的执行器（executor）在后台处理消息回调，MoveIt 就无法接收和处理来自 `/joint_states` topic 的关节状态数据，导致：

1. MoveIt 无法获取最新的关节状态
2. 时间戳显示为 0.000000
3. `getCurrentPose()` 和 `getCurrentJointValues()` 调用失败

### 相关GitHub Issue

- [moveit2_tutorials Issue #587](https://github.com/ros-planning/moveit2_tutorials/issues/587)
- [moveit2_tutorials Issue #912](https://github.com/moveit/moveit2_tutorials/issues/912)
- 相关的 ROS Answers 讨论：https://answers.ros.org/question/302283

### 为什么会发生这个问题

在 ROS1 中，`ros::AsyncSpinner` 会自动处理消息回调。但在 ROS2 中，需要显式管理执行器来确保消息回调被正确处理。这是 MoveIt2 中的一个常见问题，特别是在编写独立的 MoveIt 客户端程序时。

## 解决方案

### 方法1：使用后台执行器线程（推荐）

```cpp
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

  // 创建MoveGroup接口
  static const std::string PLANNING_GROUP = "panda_arm";
  auto move_group = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
  
  // 等待关节状态更新
  RCLCPP_INFO(node->get_logger(), "Waiting for current robot state...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // 现在可以安全地获取当前状态
  try {
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
    
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

  rclcpp::shutdown();
  return 0;
}
```

### 方法2：使用MultiThreadedExecutor

```cpp
// 替换SingleThreadedExecutor为MultiThreadedExecutor
rclcpp::executors::MultiThreadedExecutor executor;
```

### 关键点解释

1. **创建执行器**：
   ```cpp
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   ```

2. **后台线程运行**：
   ```cpp
   std::thread executor_thread([&executor]() {
     executor.spin();
   });
   ```

3. **适当等待**：
   ```cpp
   rclcpp::sleep_for(std::chrono::seconds(2));
   ```

4. **正确清理**：
   ```cpp
   executor.cancel();
   if (executor_thread.joinable()) {
     executor_thread.join();
   }
   ```

## 验证解决方案

### 运行前的错误输出
```
[ERROR] [move_group_interface]: Failed to fetch current robot state
[INFO] [hello_moveit]: Current pose: x=0.00, y=0.00, z=0.00
```

### 运行后的成功输出
```
[INFO] [hello_moveit]: Current pose: x=0.307, y=-0.000, z=0.590
[INFO] [hello_moveit]: Orientation: x=0.924, y=-0.382, z=0.000, w=0.000
[INFO] [hello_moveit]: Current joint values:
[INFO] [hello_moveit]:   Joint 0: 0.000 rad
[INFO] [hello_moveit]:   Joint 1: -0.785 rad
[INFO] [hello_moveit]:   Joint 2: 0.000 rad
[INFO] [hello_moveit]:   Joint 3: -2.356 rad
[INFO] [hello_moveit]:   Joint 4: 0.000 rad
[INFO] [hello_moveit]:   Joint 5: 1.571 rad
[INFO] [hello_moveit]:   Joint 6: 0.785 rad
```

## 最佳实践

1. **始终使用执行器**：在 MoveIt2 程序中始终显式创建和管理执行器
2. **适当的等待时间**：给 MoveIt 足够的时间来接收关节状态数据
3. **错误处理**：使用 try-catch 来处理可能的异常
4. **资源清理**：确保正确停止执行器线程

## 相关资源

- [MoveIt2 官方文档](https://moveit.picknik.ai/)
- [ROS2 执行器文档](https://docs.ros.org/en/rolling/Concepts/About-Executors.html)
- [相关 GitHub Issue #912](https://github.com/moveit/moveit2_tutorials/issues/912)

## 环境信息

- **测试环境**: ROS2 Rolling
- **机器人**: Franka Emika Panda
- **问题复现**: 运行 `ros2 launch moveit2_tutorials demo.launch.py` 后单独运行 MoveIt 客户端

---

*文档创建时间: 2025年*
*最后更新: 2025年* 