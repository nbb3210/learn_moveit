from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


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
    
    # 加载运动学配置
    kinematics_yaml = load_yaml("table_streeing_arm_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # 创建参数字典
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # 创建move_to_target节点
    move_to_target_node = Node(
        package="hello_moveit",
        executable="move_to_target",
        name="move_to_target",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )
    
    return LaunchDescription([move_to_target_node]) 