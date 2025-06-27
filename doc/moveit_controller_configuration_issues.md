# MoveIt2 控制器配置问题解决方案

本文档记录在配置 MoveIt2 控制器时遇到的常见问题及其解决方案。

## 目录

1. [执行失败：缺少 action_ns 参数](#执行失败缺少-action_ns-参数)
2. [其他控制器配置问题](#其他控制器配置问题)

---

## 执行失败：缺少 action_ns 参数

### 问题描述

在运行 `ros2 launch table_streeing_arm_config demo.launch.py` 时，规划（plan）成功但执行（execute）失败。

### 错误信息

```
[ERROR] No action namespace specified for controller `arm_group_controller` through parameter `moveit_simple_controller_manager.arm_group_controller.action_ns`
[ERROR] Unable to identify any set of controllers that can actuate the specified joints: [ joint_1 joint_2 joint_3 joint_4 joint_5 ]
[WARN] Execution request received while no controllers are active. This should never happen.
[ERROR] Controller failed during execution
```

### 问题分析

1. **缺少 action_ns 参数**：MoveIt 的 `moveit_simple_controller_manager` 需要 `action_ns` 参数来知道在哪里找到控制器的 action server
2. **控制器识别失败**：由于缺少必要的配置，MoveIt 无法识别可用的控制器
3. **执行器无法工作**：没有可用的控制器，导致轨迹执行失败

### 解决方案

在 `config/moveit_controllers.yaml` 文件中为每个控制器添加 `action_ns` 参数：

**修改前的配置**：
```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_group_controller
    - gripper_group_controller

  arm_group_controller:
    type: FollowJointTrajectory
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
  gripper_group_controller:
    type: FollowJointTrajectory
    joints:
      - joint_6
      - joint_10
      - joint_7
      - joint_11
      - joint_8
      - joint_9
```

**修改后的配置**：
```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_group_controller
    - gripper_group_controller

  arm_group_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
  gripper_group_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints:
      - joint_6
      - joint_10
      - joint_7
      - joint_11
      - joint_8
      - joint_9
```

### 关键修改

为每个控制器添加：
```yaml
action_ns: follow_joint_trajectory
```

这个参数告诉 MoveIt 控制器的 action server 在 `follow_joint_trajectory` 命名空间下可用。

### 验证修复

1. 重新启动系统：
   ```bash
   ros2 launch table_streeing_arm_config demo.launch.py
   ```

2. 在 RViz 中设置目标位置并点击 "Plan & Execute"

3. 成功的输出应该类似：
   ```
   [INFO] Planning request received for MoveGroup action. Forwarding to planning pipeline.
   [INFO] Planning pipeline completed successfully.
   [INFO] Trajectory execution succeeded
   ```

### 常见变体

根据不同的控制器类型，`action_ns` 可能有不同的值：

- **FollowJointTrajectory 控制器**：
  ```yaml
  action_ns: follow_joint_trajectory
  ```

- **夹爪控制器**：
  ```yaml
  action_ns: gripper_cmd
  ```

- **自定义控制器**：
  ```yaml
  action_ns: custom_action_namespace
  ```

---

## 其他控制器配置问题

### 控制器类型不匹配

**问题**：控制器类型配置错误导致无法连接

**解决方案**：确保 `type` 字段与实际控制器类型匹配：
```yaml
type: FollowJointTrajectory  # 对应 control_msgs/action/FollowJointTrajectory
type: GripperCommand         # 对应 control_msgs/action/GripperCommand
```

### 关节名称不匹配

**问题**：配置中的关节名称与机器人 URDF 中的名称不一致

**解决方案**：检查并确保关节名称完全匹配：
```bash
# 查看当前关节状态
ros2 topic echo --once /joint_states

# 检查 URDF 中的关节名称
ros2 param get /robot_state_publisher robot_description
```

### 控制器管理器配置错误

**问题**：错误的控制器管理器类型

**解决方案**：根据使用的控制器系统选择正确的管理器：
```yaml
# 简单控制器管理器（适用于大多数情况）
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

# ros2_control 接口
moveit_controller_manager: moveit_ros_control_interface::Ros2ControlManager
```

---

## 最佳实践

1. **始终包含 action_ns**：即使对于默认命名空间，也显式指定 `action_ns`
2. **验证关节名称**：确保配置中的关节名称与 URDF 完全匹配
3. **测试单个控制器**：使用 `ros2 action` 命令单独测试每个控制器
4. **检查控制器状态**：使用控制器管理器服务检查控制器是否正常运行

---

## 故障排除命令

```bash
# 检查可用的 action 服务器
ros2 action list

# 测试控制器连接
ros2 action info /follow_joint_trajectory

# 检查控制器管理器
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# 查看 MoveIt 日志
ros2 topic echo /move_group/display_planned_path
```

---

*文档创建时间: 2025年6月25日*  
*最后更新: 2025年6月25日* 