#include "diffdrive_arduino/DiffDriveArduinoHardwareInterface.hpp"

namespace diffdrive_arduino
{

DiffDriveArduinoHardwareInterface::DiffDriveArduinoHardwareInterface()
  : serial_port_(std::make_unique<boost::asio::serial_port>(io_service_)) {}  

hardware_interface::CallbackReturn DiffDriveArduinoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  hw_commands_.resize(4, 0.0);
  hw_states_.resize(8, 0.0);

  command_publisher_ = rclcpp::Node::make_shared("diffdrive_arduino_node")->create_publisher<msg_utils::msg::WheelCommands>("cmd_vel_published", 10);
  //state_publisher_ = rclcpp::Node::make_shared("diffdrive_arduino_node")->create_publisher<geometry_msgs::msg::Twist>("wheel_states_published", 10);

  // Open Serial port
  try {
    serial_port_->open("/dev/ttyUSB0");
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(9600));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  } catch (boost::system::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Erreur lors de l'ouverture du port série: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", "position", &hw_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", "velocity", &hw_states_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", "position", &hw_states_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", "velocity", &hw_states_[5]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rear_right_wheel_joint", "position", &hw_states_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rear_right_wheel_joint", "velocity", &hw_states_[6]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rear_left_wheel_joint", "position", &hw_states_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rear_left_wheel_joint", "velocity", &hw_states_[7]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface("front_right_wheel_joint", "velocity", &hw_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("front_left_wheel_joint", "velocity", &hw_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("rear_right_wheel_joint", "velocity", &hw_commands_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("rear_left_wheel_joint", "velocity", &hw_commands_[3]));
  return command_interfaces;
}

hardware_interface::return_type DiffDriveArduinoHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read data from Arduino
  try {
    boost::asio::streambuf response;
    boost::asio::read_until(*serial_port_, response, "\n");
    std::istream response_stream(&response);

    // Parse values for position and speed
    response_stream >> hw_states_[0] >> hw_states_[1] >> hw_states_[2] >> hw_states_[3];
  } catch (boost::system::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Erreur de lecture: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveArduinoHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Commands are sent to motors
  std::ostringstream command;
  command << hw_commands_[0] << "," << hw_commands_[1] << "," << hw_commands_[2] << "," << hw_commands_[3] << "\n";

  try {
    boost::asio::write(*serial_port_, boost::asio::buffer(command.str()));
  } catch (boost::system::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Erreur d'écriture: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  msg_utils::msg::WheelCommands cmd_msg;
  cmd_msg.front_left_wheel_speed.data = hw_commands_[1]; 
  cmd_msg.front_right_wheel_speed.data = hw_commands_[0];
  cmd_msg.rear_right_wheel_speed.data = hw_commands_[2]; 
  cmd_msg.rear_left_wheel_speed.data = hw_commands_[3];

  command_publisher_->publish(cmd_msg);


  return hardware_interface::return_type::OK;
}

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardwareInterface, hardware_interface::SystemInterface)
