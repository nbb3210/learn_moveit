#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/wheeltec_controller",
            description="Serial port for the robot arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Baudrate for serial communication",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("table_streeing_arm_config"),
                    "config",
                    "table_streeing_arm_real_hardware.urdf.xacro",
                ]
            ),
            " serial_port:=",
            serial_port,
            " baudrate:=",
            baudrate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get ros2_control yaml file
    ros2_controllers_path = os.path.join(
        get_package_share_directory("table_streeing_arm_config"),
        "config",
        "ros2_controllers_real_hardware.yaml",
    )

    # Start the actual move_group node/action server
    moveit_config = MoveItConfigsBuilder("table_streeing_arm", package_name="table_streeing_arm_config").to_moveit_configs()

    # ros2_control using real hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Start controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
    )

    # gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_group_controller", "--controller-manager", "/controller_manager"],
    # )

    # MoveIt move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("table_streeing_arm_config"), "rviz", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        # gripper_controller_spawner,
        run_move_group_node,
        rviz_node,
    ]

    return nodes_to_start 