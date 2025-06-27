#!/usr/bin/env python3
"""
简单的夹爪控制脚本
提供打开和关闭夹爪的便捷命令
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 发布器 - 发送夹爪位置命令
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/gripper_controller/commands', 
            10
        )
        
        # 夹爪位置定义
        self.GRIPPER_OPEN = 0.75      # 完全张开
        self.GRIPPER_CLOSED = -1.15   # 完全闭合
        
        self.get_logger().info('夹爪控制器已启动')
        self.print_usage()
    
    def print_usage(self):
        """打印使用说明"""
        print("\n=== 夹爪控制器 ===")
        print("使用方法:")
        print("  python3 gripper_control.py open   # 打开夹爪")
        print("  python3 gripper_control.py close  # 关闭夹爪")
        print("  python3 gripper_control.py <value> # 设置特定位置 (0.75到-1.15)")
        print("==================\n")
    
    def open_gripper(self):
        """打开夹爪"""
        msg = Float64MultiArray()
        msg.data = [self.GRIPPER_OPEN]
        self.publisher.publish(msg)
        self.get_logger().info(f'夹爪打开: {msg.data[0]}')
    
    def close_gripper(self):
        """关闭夹爪"""
        msg = Float64MultiArray()
        msg.data = [self.GRIPPER_CLOSED]
        self.publisher.publish(msg)
        self.get_logger().info(f'夹爪关闭: {msg.data[0]}')
    
    def set_gripper_position(self, position):
        """设置夹爪到指定位置"""
        # 限制范围在-1.15到0.75之间
        position = max(-1.15, min(0.75, position))
        
        msg = Float64MultiArray()
        msg.data = [position]
        self.publisher.publish(msg)
        self.get_logger().info(f'夹爪位置设置为: {msg.data[0]}')


def main():
    rclpy.init()
    
    gripper = GripperController()
    
    if len(sys.argv) < 2:
        gripper.print_usage()
        return
    
    command = sys.argv[1].lower()
    
    try:
        if command == 'open':
            gripper.open_gripper()
        elif command == 'close':
            gripper.close_gripper()
        else:
            # 尝试将输入作为数值
            try:
                position = float(command)
                gripper.set_gripper_position(position)
            except ValueError:
                print(f"未知命令: {command}")
                gripper.print_usage()
                return
        
        # 让节点运行一小段时间以确保消息发送
        rclpy.spin_once(gripper, timeout_sec=1.0)
        
    except Exception as e:
        gripper.get_logger().error(f'错误: {e}')
    finally:
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 