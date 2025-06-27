#ifndef TABLE_STEERING_ARM_HARDWARE__TABLE_STEERING_ARM_HARDWARE_HPP_
#define TABLE_STEERING_ARM_HARDWARE__TABLE_STEERING_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Serial communication
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace table_steering_arm_hardware
{
class TableSteeringArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TableSteeringArmHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string serial_port_;
  int baudrate_;
  
  // Protocol constants
  static constexpr uint8_t FRAME_HEADER_SEND = 0xAA;
  static constexpr uint8_t FRAME_TAIL_SEND = 0xBB;
  static constexpr uint8_t FRAME_HEADER_RECV = 0x7B;
  static constexpr uint8_t FRAME_TAIL_RECV = 0x7D;
  static constexpr uint8_t MODE_DEFAULT = 1;
  static constexpr uint8_t MODE_FOLLOWER = 2;
  
  // Joint limits (radians)
  static constexpr double ANGLE_MIN = -1.57;
  static constexpr double ANGLE_MAX = 1.57;
  static constexpr double ANGLE_4_MIN = -0.45;
  static constexpr double ANGLE_6_MIN = -1.15;
  static constexpr double ANGLE_6_MAX = 0.75;

  // Hardware interface data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> prev_positions_;

  // Serial communication
  int serial_fd_;
  std::atomic<bool> is_connected_;
  
  // Reading thread
  std::thread read_thread_;
  std::atomic<bool> read_thread_running_;
  std::mutex data_mutex_;
  
  // Communication methods
  bool connect();
  void disconnect();
  bool send_joint_angles(const std::vector<double>& angles);
  void read_data_loop();
  bool parse_feedback_packet(const std::vector<uint8_t>& packet);
  uint8_t calculate_checksum(const std::vector<uint8_t>& data);
  double limit_angle(double angle, int joint_index);
  
  // Velocity estimation
  rclcpp::Time last_read_time_;
  bool first_read_;
};

}  // namespace table_steering_arm_hardware

#endif  // TABLE_STEERING_ARM_HARDWARE__TABLE_STEERING_ARM_HARDWARE_HPP_ 