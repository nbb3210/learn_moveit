from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    robot_description_pkg = get_package_share_directory('table_streeing_arm')
    moveit_config_pkg = get_package_share_directory('table_streeing_arm_config')
    
    # 读取URDF文件
    urdf_file = os.path.join(robot_description_pkg, 'urdf', 'table_streeing_arm.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # 读取SRDF文件
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'table_streeing_arm.srdf')
    with open(srdf_file, 'r') as infp:
        robot_description_semantic_content = infp.read()
    
    # 创建参数字典
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # 创建hello_moveit节点
    hello_moveit_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        name="hello_moveit",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )
    
    return LaunchDescription([hello_moveit_node]) 