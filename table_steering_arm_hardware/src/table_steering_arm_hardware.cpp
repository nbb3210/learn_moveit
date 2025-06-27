#include "table_steering_arm_hardware/table_steering_arm_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace table_steering_arm_hardware
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("TableSteeringArmHardware");

hardware_interface::CallbackReturn TableSteeringArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  serial_port_ = info_.hardware_parameters["serial_port"];
  baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

  // Initialize hardware interface vectors
  hw_commands_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size());
  hw_velocities_.resize(info_.joints.size());
  prev_positions_.resize(info_.joints.size());

  // Initialize all values to zero
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_commands_[i] = 0.0;
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    prev_positions_[i] = 0.0;
  }

  // Initialize flags
  serial_fd_ = -1;
  is_connected_ = false;
  read_thread_running_ = false;
  first_read_ = true;

  RCLCPP_INFO(LOGGER, "Successfully initialized TableSteeringArmHardware with serial port: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TableSteeringArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TableSteeringArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TableSteeringArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TableSteeringArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating hardware interface...");
  
  if (!connect()) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start reading thread
  read_thread_running_ = true;
  read_thread_ = std::thread(&TableSteeringArmHardware::read_data_loop, this);

  // Initialize commands to current positions
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    hw_commands_ = hw_positions_;
  }

  RCLCPP_INFO(LOGGER, "Hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TableSteeringArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating hardware interface...");
  
  // Stop reading thread
  read_thread_running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  disconnect();
  
  RCLCPP_INFO(LOGGER, "Hardware interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TableSteeringArmHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Data is read in the background thread, just update velocities here
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (first_read_) {
    last_read_time_ = time;
    first_read_ = false;
    return hardware_interface::return_type::OK;
  }

  // Calculate velocities using finite difference
  auto dt = time - last_read_time_;
  double dt_sec = dt.seconds();
  
  if (dt_sec > 0.0) {
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt_sec;
      prev_positions_[i] = hw_positions_[i];
    }
  }
  
  last_read_time_ = time;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TableSteeringArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!is_connected_) {
    return hardware_interface::return_type::ERROR;
  }

  // Send joint angles to the arm
  if (!send_joint_angles(hw_commands_)) {
    RCLCPP_WARN(LOGGER, "Failed to send joint angles");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool TableSteeringArmHardware::connect()
{
  try {
    // Open serial port
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(LOGGER, "Failed to open serial port %s", serial_port_.c_str());
      return false;
    }

    // Configure serial port
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(LOGGER, "Error getting serial port attributes");
      close(serial_fd_);
      return false;
    }

    // Set baud rate
    speed_t speed = B115200;  // Default to 115200
    switch (baudrate_) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default:
        RCLCPP_WARN(LOGGER, "Unsupported baud rate %d, using 115200", baudrate_);
        break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1
    tty.c_cflag &= ~PARENB;   // Clear parity bit
    tty.c_cflag &= ~CSTOPB;   // Clear stop field, only one stop bit used
    tty.c_cflag &= ~CSIZE;    // Clear all bits that set the data size
    tty.c_cflag |= CS8;       // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;   // Disable canonical mode
    tty.c_lflag &= ~ECHO;     // Disable echo
    tty.c_lflag &= ~ECHOE;    // Disable erasure
    tty.c_lflag &= ~ECHONL;   // Disable new-line echo
    tty.c_lflag &= ~ISIG;     // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;   // Wait for up to 1s (10 deciseconds), returning as soon as any data is received
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(LOGGER, "Error setting serial port attributes");
      close(serial_fd_);
      return false;
    }

    is_connected_ = true;
    RCLCPP_INFO(LOGGER, "Successfully connected to serial port %s at %d baud", serial_port_.c_str(), baudrate_);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to serial port: %s", e.what());
    is_connected_ = false;
    return false;
  }
}

void TableSteeringArmHardware::disconnect()
{
  is_connected_ = false;
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(LOGGER, "Serial port disconnected");
  }
}

bool TableSteeringArmHardware::send_joint_angles(const std::vector<double>& angles)
{
  if (!is_connected_ || angles.size() != 6) {
    return false;
  }

  try {
    // Construct data packet (16 bytes)
    std::vector<uint8_t> packet(16);
    packet[0] = FRAME_HEADER_SEND;
    
    // Pack 6 joint angles (convert radians to 0.001 radian units)
    for (size_t i = 0; i < 6; i++) {
      double limited_angle = limit_angle(angles[i], i);
      int16_t angle_int = static_cast<int16_t>(limited_angle * 1000.0);
      
      // Handle negative numbers for 16-bit representation
      uint16_t angle_uint = static_cast<uint16_t>(angle_int);
      
      packet[1 + i*2] = (angle_uint >> 8) & 0xFF;  // High byte
      packet[2 + i*2] = angle_uint & 0xFF;         // Low byte
    }
    
    packet[13] = MODE_DEFAULT;  // Control mode
    packet[14] = calculate_checksum(std::vector<uint8_t>(packet.begin(), packet.begin() + 14));
    packet[15] = FRAME_TAIL_SEND;
    
    // Send data
    ssize_t bytes_written = ::write(serial_fd_, packet.data(), packet.size());
    if (bytes_written != static_cast<ssize_t>(packet.size())) {
      RCLCPP_ERROR(LOGGER, "Failed to write all bytes to serial port");
      return false;
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to send joint angles: %s", e.what());
    return false;
  }
}

void TableSteeringArmHardware::read_data_loop()
{
  std::vector<uint8_t> buffer;
  uint8_t read_buffer[256];
  
  while (read_thread_running_ && is_connected_) {
    try {
      // Read available data
      ssize_t bytes_read = ::read(serial_fd_, read_buffer, sizeof(read_buffer));
      if (bytes_read > 0) {
        buffer.insert(buffer.end(), read_buffer, read_buffer + bytes_read);
        
        // Look for complete data packets
        while (buffer.size() >= 16) {
          // Find frame header
          auto header_it = std::find(buffer.begin(), buffer.end(), FRAME_HEADER_RECV);
          if (header_it == buffer.end()) {
            buffer.clear();
            break;
          }
          
          // Remove data before header
          if (header_it != buffer.begin()) {
            buffer.erase(buffer.begin(), header_it);
          }
          
          // Check if we have a complete 16-byte packet
          if (buffer.size() >= 16) {
            std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 16);
            
            // Verify frame tail and checksum
            if (packet[15] == FRAME_TAIL_RECV) {
              uint8_t checksum = calculate_checksum(std::vector<uint8_t>(packet.begin(), packet.begin() + 14));
              if (checksum == packet[14]) {
                parse_feedback_packet(packet);
              }
            }
            
            // Remove processed packet
            buffer.erase(buffer.begin(), buffer.begin() + 16);
          } else {
            break;
          }
        }
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Error in read data loop: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

bool TableSteeringArmHardware::parse_feedback_packet(const std::vector<uint8_t>& packet)
{
  try {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Parse 6 joint angles from feedback
    for (size_t i = 0; i < 6; i++) {
      uint8_t high_byte = packet[1 + i*2];
      uint8_t low_byte = packet[2 + i*2];
      uint16_t angle_uint = (high_byte << 8) | low_byte;
      
      // Handle negative numbers (16-bit signed integer)
      int16_t angle_int = static_cast<int16_t>(angle_uint);
      
      // Convert to radians (assuming feedback precision of 0.01)
      hw_positions_[i] = angle_int * 0.01;
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to parse feedback packet: %s", e.what());
    return false;
  }
}

uint8_t TableSteeringArmHardware::calculate_checksum(const std::vector<uint8_t>& data)
{
  uint8_t checksum = 0;
  for (const auto& byte : data) {
    checksum ^= byte;
  }
  return checksum;
}

double TableSteeringArmHardware::limit_angle(double angle, int joint_index)
{
  switch (joint_index) {
    case 3:  // Joint 4 special limits
      return std::max(ANGLE_4_MIN, std::min(ANGLE_MAX, angle));
    case 5:  // Joint 6 special limits
      return std::max(ANGLE_6_MIN, std::min(ANGLE_6_MAX, angle));
    default:
      return std::max(ANGLE_MIN, std::min(ANGLE_MAX, angle));
  }
}

}  // namespace table_steering_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  table_steering_arm_hardware::TableSteeringArmHardware, hardware_interface::SystemInterface) 