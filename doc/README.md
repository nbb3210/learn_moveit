# MoveIt2 问题解决方案文档

这个目录包含了在使用 MoveIt2 过程中遇到的常见问题及其解决方案。

## 文档列表

### [failed_to_fetch_robot_state_issue.md](./failed_to_fetch_robot_state_issue.md)

**问题**: "Failed to fetch current robot state" 错误

**简述**: 在调用 `move_group.getCurrentPose()` 时出现错误，无法获取机器人当前状态。

**解决方案**: 使用后台执行器线程来确保 ROS2 消息回调正确处理。

**关键代码**:
```cpp
// 创建执行器并在后台线程运行
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread executor_thread([&executor]() {
  executor.spin();
});
```

### [robot_control_methods.md](./robot_control_methods.md)

**主题**: MoveIt2 机械臂控制方法全解析

**简述**: 详细介绍了在 MoveIt2 环境中控制 Panda 机械臂的所有可用方法，包括底层关节控制、中层路径规划、高层任务控制等。

**内容涵盖**:
- 直接关节控制 (Topic & Action)
- MoveIt 高级规划接口
- 位姿目标控制
- 运动学服务 (FK/IK/路径规划)
- 夹爪控制
- 交互式控制和可视化
- 控制器管理
- 预定义位置和最佳实践

### [moveit_controller_configuration_issues.md](./moveit_controller_configuration_issues.md)

**问题**: MoveIt2 控制器配置问题

**简述**: 记录控制器配置相关的常见问题，特别是缺少 `action_ns` 参数导致轨迹执行失败的问题。

**解决方案**: 在 `moveit_controllers.yaml` 中为每个控制器添加正确的 `action_ns` 参数。

**关键修改**:
```yaml
arm_group_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory  # 添加此行
  joints: [...]
```

### [table_streeing_arm_config_analysis.md](./table_streeing_arm_config_analysis.md)

**主题**: table_streeing_arm_config 包分析

**简述**: 详细分析 `table_streeing_arm_config` MoveIt 配置包的结构、功能和使用方法。

**内容涵盖**:
- 包的整体架构和自动生成过程
- SRDF 文件中的运动组和预定义姿态
- 各种配置文件的作用和参数说明
- 启动文件的功能和使用方法
- 与用户代码的集成方式
- 配置定制和故障排除指南

**关键信息**:
- 机械臂组: `arm_group` (joint_1 到 joint_5)
- 夹爪组: `gripper_group` (joint_6 到 joint_11)
- 预定义姿态: `home` (全零位置), `fold` (折叠姿态)

### [table_streeing_arm_package_analysis.md](./table_streeing_arm_package_analysis.md)

**主题**: table_streeing_arm 包分析及其与 table_streeing_arm_config 的关系

**简述**: 详细分析 `table_streeing_arm` 机器人描述包的结构和功能，以及它与 `table_streeing_arm_config` MoveIt 配置包的层次关系。

**内容涵盖**:
- 机器人 URDF 结构和关节定义（11个关节）
- 3D模型文件和物理属性配置
- 两个包之间的依赖关系和功能分工
- 基础可视化 vs MoveIt 运动规划的使用场景
- 开发定制指南和故障排除

**关键概念**:
- 基础设施包 vs 应用层包的架构设计
- URDF → SRDF → MoveIt 的数据流关系
- 分层式机器人软件架构的最佳实践

### [project_progress_summary.md](./project_progress_summary.md)

**主题**: A150 C06B 机械臂 ROS2 Control + MoveIt2 项目完整进展总结

**简述**: 记录整个项目从零开始到完成的完整过程，包括所有技术实现、问题解决和最终成果。

**项目成果**:
- ✅ 完整的ROS2 Control硬件接口实现
- ✅ 机械臂轨迹控制和夹爪位置控制
- ✅ MoveIt2运动规划集成
- ✅ 用户友好的控制工具
- ✅ 完整的系统文档

**技术亮点**:
- 自主实现WHEELTEC串口通信协议
- 6关节位置控制和实时状态反馈
- 分离式控制架构（机械臂+夹爪）
- 无外部依赖的硬件接口设计
- 完整的MoveIt2集成和可视化

**使用指南**:
- 系统启动和配置方法
- 机械臂和夹爪控制命令
- 故障排除和注意事项

---

## 如何使用这些文档

1. 当遇到 MoveIt2 相关问题时，首先查看对应的问题文档
2. 按照文档中的解决方案进行修复
3. 如果遇到新问题，请添加新的文档到此目录

## 贡献指南

- 文档格式使用 Markdown
- 包含完整的问题描述、分析和解决方案
- 提供可运行的代码示例
- 包含验证步骤和预期输出

---

## 🎉 项目里程碑

### 2025年1月5日 - 项目基本完成 ✅
- 完成了A150 C06B机械臂的完整ROS2 Control + MoveIt2控制系统
- 实现了机械臂轨迹控制和夹爪位置控制
- 所有核心功能测试通过，系统可投入实际使用

### 主要技术成就
1. **硬件接口**: 自主开发的ROS2 Control硬件接口包
2. **通信协议**: 完整实现WHEELTEC串口通信协议
3. **控制架构**: 分离式机械臂和夹爪控制设计
4. **系统集成**: 完整的MoveIt2运动规划和可视化
5. **用户工具**: 提供便捷的Python控制脚本

---

*项目完成: 2025年1月5日*  
*最后更新: 2025年1月5日* 