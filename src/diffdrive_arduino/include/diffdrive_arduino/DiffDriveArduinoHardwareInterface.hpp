#ifndef DIFFDRIVE_ARDUINO_HARDWARE_INTERFACE_HPP
#define DIFFDRIVE_ARDUINO_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp> // ROS 2 hardware interface header
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp> // Boost library for serial communicatio
#include <vector>
#include <string>
#include "msg_utils/msg/wheel_commands.hpp" // Custom message type for wheel commands


namespace diffdrive_arduino
{

// Class that implements the hardware interface for a differential drive robot using Arduino
class DiffDriveArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  DiffDriveArduinoHardwareInterface();  
  ~DiffDriveArduinoHardwareInterface() noexcept override = default;

  // Initializes the hardware interface with information from the configuration file
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Exports the state interfaces, which provide access to the robot's state
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Exports the command interfaces, which allow sending commands to the robot
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Reads data from the hardware (e.g., encoders, sensors) and updates the hardware state
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Writes commands to the hardware (e.g., motor speeds) based on the current commands
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  rclcpp::Publisher<msg_utils::msg::WheelCommands>::SharedPtr command_publisher_;
  //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr state_publisher_;
  std::map<std::string, double> hw_commands_; 
  std::map<std::string, std::array<double, 2>> hw_states_;
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::serial_port> serial_port_; 
};

} // namespace diffdrive_arduino

#endif // DIFFDRIVE_ARDUINO_HARDWARE_INTERFACE_HPP
