# Table Steering Arm Hardware Interface

This package provides a ROS2 Control hardware interface for the A150 C06B table steering arm robot, enabling seamless integration with MoveIt2.

## Features

- **Serial Communication**: Communicates with STM32 controller via UART/RS232
- **6-DOF Control**: Controls 6 joints of the robotic arm
- **Real-time Feedback**: Reads joint positions and estimates velocities
- **MoveIt2 Integration**: Fully compatible with MoveIt2 motion planning
- **Safety Limits**: Enforces joint angle limits based on hardware specifications

## Hardware Protocol

The interface implements the WHEELTEC robotic arm communication protocol:

- **Baud Rate**: 115200
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Packet Size**: 16 bytes
- **Frame Header**: 0xAA (send), 0x7B (receive)
- **Frame Tail**: 0xBB (send), 0x7D (receive)
- **Checksum**: XOR of all data bytes

### Packet Structure (Send)
```
[0xAA][Joint1_H][Joint1_L][Joint2_H][Joint2_L][Joint3_H][Joint3_L]
[Joint4_H][Joint4_L][Joint5_H][Joint5_L][Joint6_H][Joint6_L][Mode][Checksum][0xBB]
```

### Joint Limits
- **Joint 1-3, 5**: -1.57 to 1.57 radians (-90° to 90°)
- **Joint 4**: -0.45 to 1.57 radians (-25.7° to 90°)
- **Joint 6**: -1.15 to 0.75 radians (-65.9° to 43.0°)

## Installation

1. **Install Dependencies**:
   ```bash
   sudo apt install libserial-dev
   ```

2. **Build the Package**:
   ```bash
   cd ~/ws_moveit
   colcon build --packages-select table_steering_arm_hardware
   source install/setup.bash
   ```

## Usage

### Basic Usage with ros2_control

1. **Configure URDF**: Use the provided ros2_control configuration:
   ```xml
   <ros2_control name="TableSteeringArmHardware" type="system">
     <hardware>
       <plugin>table_steering_arm_hardware/TableSteeringArmHardware</plugin>
       <param name="serial_port">/dev/wheeltec_controller</param>
       <param name="baudrate">115200</param>
     </hardware>
     <!-- Joint configurations... -->
   </ros2_control>
   ```

2. **Launch with Real Hardware**:
   ```bash
   ros2 launch table_streeing_arm_config real_hardware_demo.launch.py
   ```

3. **Custom Serial Port**:
   ```bash
   ros2 launch table_streeing_arm_config real_hardware_demo.launch.py serial_port:=/dev/ttyUSB0
   ```

### Integration with MoveIt2

The hardware interface is designed to work seamlessly with MoveIt2:

1. **Load Controllers**:
   ```bash
   ros2 control load_controller arm_group_controller
   ros2 control set_controller_state arm_group_controller active
   ```

2. **Use with MoveIt2**:
   - The interface provides position command interfaces
   - Supports FollowJointTrajectory action
   - Compatible with MoveIt motion planning

### Manual Control

You can also control individual joints:

```bash
# Send position commands
ros2 topic pub /arm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
points:
- positions: [0.0, 0.5, -0.3, 0.2, 0.4, 0.1]
  time_from_start: {sec: 2}
"
```

## Configuration Files

### For Real Hardware
- **URDF**: `config/table_streeing_arm_real_hardware.urdf.xacro`
- **Controllers**: `config/ros2_controllers_real_hardware.yaml`
- **MoveIt**: `config/moveit_controllers_real_hardware.yaml`

### Parameters
- `serial_port`: Serial port device (default: `/dev/wheeltec_controller`)
- `baudrate`: Communication baud rate (default: `115200`)

## Troubleshooting

### Serial Port Issues
1. **Permission Denied**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

2. **Device Not Found**:
   ```bash
   # List available serial ports
   ls /dev/tty*
   # Check if device is connected
   dmesg | grep tty
   ```

### Connection Issues
1. **Check cables**: Ensure proper UART/RS232 connection
2. **Verify baud rate**: Must match STM32 controller settings (115200)
3. **Test with Python controller**: Use the provided Python scripts to verify hardware

### Controller Issues
1. **Check controller status**:
   ```bash
   ros2 control list_controllers
   ```

2. **Monitor topics**:
   ```bash
   ros2 topic echo /joint_states
   ```

## Architecture

```
MoveIt2 → FollowJointTrajectory → JointTrajectoryController → Hardware Interface → Serial Port → STM32
```

The hardware interface acts as a bridge between ROS2 Control and the physical robot, handling:
- Serial communication protocol
- Data format conversion
- Joint angle limiting
- Velocity estimation
- Error handling

## Development

### Adding New Features
1. Modify the hardware interface class in `src/table_steering_arm_hardware.cpp`
2. Update the header file in `include/table_steering_arm_hardware/table_steering_arm_hardware.hpp`
3. Rebuild and test

### Testing
1. **Simulation**: Use `mock_components/GenericSystem` for testing without hardware
2. **Hardware-in-loop**: Connect to real STM32 controller
3. **Unit tests**: Implement using ROS2 testing framework

## License

Apache License 2.0

## Support

For issues and questions:
1. Check the troubleshooting section
2. Verify hardware connections
3. Test with provided Python scripts
4. Open an issue with detailed error logs 