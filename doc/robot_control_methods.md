# MoveIt2 机械臂控制方法全解析

本文档详细介绍了在 MoveIt2 环境中控制 Panda 机械臂的所有可用方法。

## 控制层级架构

```
高层控制 (任务级)
    ↓
中层控制 (路径规划)
    ↓
底层控制 (关节轨迹)
    ↓
硬件接口 (电机控制)
```

## 1. 直接关节控制 (底层)

### 1.1 Topic 方式控制

**优点**: 简单直接，延迟低  
**缺点**: 无执行反馈，可能丢失消息

```bash
# 基本语法
ros2 topic pub [选项] /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory [消息内容]

# 移动到 ready 位置
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
points:
- positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  time_from_start: {sec: 5, nanosec: 0}"

# 移动到 extended 位置
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
points:
- positions: [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785]
  time_from_start: {sec: 4, nanosec: 0}"

# 自定义关节位置（小幅移动第一个关节）
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
points:
- positions: [0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start: {sec: 3, nanosec: 0}"
```

### 1.2 Action 方式控制 (推荐)

**优点**: 有执行反馈，可以监控状态，更可靠  
**缺点**: 稍微复杂一些

```bash
# 基本语法
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory [目标]

# 实际示例
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  header:
    stamp: {sec: 0, nanosec: 0}
    frame_id: ''
  joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
  points:
  - positions: [0.3, -0.5, 0.2, -1.8, 0.1, 1.2, 0.9]
    time_from_start: {sec: 3, nanosec: 0}
goal_tolerance:
- name: 'panda_joint1'
  position: 0.01
- name: 'panda_joint2'  
  position: 0.01"
```

## 2. MoveIt 高级规划接口 (中层)

### 2.1 完整规划和执行

```bash
# MoveIt 的完整 move_action (规划 + 执行)
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "
request:
  group_name: 'panda_arm'
  max_velocity_scaling_factor: 0.1
  max_acceleration_scaling_factor: 0.1
  goal_constraints:
  - joint_constraints:
    - joint_name: 'panda_joint1'
      position: 0.0
      tolerance_above: 0.01
      tolerance_below: 0.01
      weight: 1.0"
```

### 2.2 仅执行预计算轨迹

```bash
# 执行已规划的轨迹
ros2 action send_goal /execute_trajectory moveit_msgs/action/ExecuteTrajectory "
trajectory:
  joint_trajectory:
    header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: ''
    joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    points:
    - positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
      time_from_start: {sec: 5, nanosec: 0}"
```

## 3. 位姿目标控制 (高层)

### 3.1 目标位姿设置

```bash
# 通过目标位姿控制（配合 RViz 使用）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'panda_link0'
pose:
  position: {x: 0.4, y: 0.1, z: 0.4}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# 设置初始位姿
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'panda_link0'
pose:
  pose:
    position: {x: 0.3, y: 0.0, z: 0.6}
    orientation: {x: 0.924, y: -0.382, z: 0.0, w: 0.0}"
```

## 4. MoveIt 服务接口

### 4.1 运动学服务

```bash
# 运动学路径规划
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "
motion_plan_request:
  group_name: 'panda_arm'
  num_planning_attempts: 10
  max_velocity_scaling_factor: 0.1
  max_acceleration_scaling_factor: 0.1"

# 正向运动学计算
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'panda_link0'
fk_link_names: ['panda_link8']
robot_state:
  joint_state:
    name: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

# 逆向运动学计算
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
ik_request:
  group_name: 'panda_arm'
  pose_stamped:
    header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: 'panda_link0'
    pose:
      position: {x: 0.307, y: 0.0, z: 0.590}
      orientation: {x: 0.924, y: -0.382, z: 0.0, w: 0.0}"

# 笛卡尔路径计算
ros2 service call /compute_cartesian_path moveit_msgs/srv/GetCartesianPath "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'panda_link0'
group_name: 'panda_arm'
link_name: 'panda_link8'
waypoints:
- position: {x: 0.3, y: 0.0, z: 0.6}
  orientation: {x: 0.924, y: -0.382, z: 0.0, w: 0.0}
- position: {x: 0.4, y: 0.1, z: 0.5}
  orientation: {x: 0.924, y: -0.382, z: 0.0, w: 0.0}
max_step: 0.01
jump_threshold: 0.0"
```

### 4.2 规划场景管理

```bash
# 获取当前规划场景
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "
components:
  components: 1023"  # 获取所有组件

# 应用规划场景修改
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "
scene:
  name: 'updated_scene'
  robot_state:
    joint_state:
      name: ['panda_joint1']
      position: [0.5]"
```

## 5. 夹爪控制

```bash
# 打开夹爪 (0.04m 是最大开度)
ros2 action send_goal /panda_hand_controller/gripper_cmd control_msgs/action/GripperCommand "
command:
  position: 0.04
  max_effort: 60.0"

# 关闭夹爪
ros2 action send_goal /panda_hand_controller/gripper_cmd control_msgs/action/GripperCommand "
command:
  position: 0.0
  max_effort: 60.0"

# 部分关闭 (抓取物体)
ros2 action send_goal /panda_hand_controller/gripper_cmd control_msgs/action/GripperCommand "
command:
  position: 0.02
  max_effort: 30.0"
```

## 6. 交互式控制

### 6.1 RViz 交互式标记

```bash
# 监听交互式标记反馈
ros2 topic echo /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback

# 获取交互式标记
ros2 service call /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers visualization_msgs/srv/GetInteractiveMarkers
```

## 7. 控制器管理

```bash
# 列出所有控制器
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# 查看控制器状态
ros2 service call /panda_arm_controller/query_state controller_manager_msgs/srv/QueryControllerStates

# 切换控制器
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "
start_controllers: ['panda_arm_controller']
stop_controllers: []
strictness: 2"
```

## 8. 实用工具命令

### 8.1 状态查询

```bash
# 查看当前关节状态
ros2 topic echo --once /joint_states

# 查看规划场景
ros2 topic echo --once /monitored_planning_scene

# 查看轨迹执行事件
ros2 topic echo /trajectory_execution_event
```

### 8.2 参数配置

```bash
# 获取规划器参数
ros2 service call /get_planner_params moveit_msgs/srv/GetPlannerParams "
planner_config: 'ompl'"

# 设置规划器参数
ros2 service call /set_planner_params moveit_msgs/srv/SetPlannerParams "
planner_config: 'ompl'
group: 'panda_arm'
params:
  planning_time: '5.0'
  planning_attempts: '10'"

# 查询规划器接口
ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces
```

## 9. 预定义位置

基于 SRDF 文件中的定义，以下是一些预设位置：

```bash
# Ready 位置 (准备姿态)
positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Extended 位置 (伸展姿态)  
positions: [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785]

# Transport 位置 (运输姿态)
positions: [0.0, -0.5599, 0.0, -2.97, 0.0, 0.0, 0.785]
```

## 10. 最佳实践

### 10.1 选择控制方法的指南

- **简单位置控制**: 使用关节轨迹 topic
- **需要反馈**: 使用 action 接口
- **复杂路径规划**: 使用 MoveIt 服务
- **避障要求**: 使用 MoveIt move_action
- **实时交互**: 使用 RViz 交互式标记
- **精确控制**: 使用笛卡尔路径规划

### 10.2 安全注意事项

1. **速度限制**: 始终设置合理的速度缩放因子 (0.1-0.5)
2. **平滑运动**: 使用时间参数确保平滑过渡
3. **碰撞检测**: 在复杂环境中使用 MoveIt 规划
4. **状态验证**: 执行前检查机械臂当前状态

### 10.3 故障排除

```bash
# 检查控制器状态
ros2 control list_controllers

# 检查话题连接
ros2 topic info /panda_arm_controller/joint_trajectory

# 检查 action 服务器
ros2 action info /panda_arm_controller/follow_joint_trajectory

# 重启控制器
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "
stop_controllers: ['panda_arm_controller']
start_controllers: ['panda_arm_controller']
strictness: 2"
```

---

## 环境信息

- **ROS版本**: ROS2 Rolling
- **机器人**: Franka Emika Panda  
- **控制器**: `panda_arm_controller`, `panda_hand_controller`
- **规划组**: `panda_arm`, `hand`, `panda_arm_hand`

---

*文档创建时间: 2025年*  
*最后更新: 2025年* 