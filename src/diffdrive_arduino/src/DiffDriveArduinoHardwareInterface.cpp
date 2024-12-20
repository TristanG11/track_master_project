#include "diffdrive_arduino/DiffDriveArduinoHardwareInterface.hpp"
#include <iomanip> // Pour std::setprecision
namespace diffdrive_arduino
{

DiffDriveArduinoHardwareInterface::DiffDriveArduinoHardwareInterface()
{}  

hardware_interface::CallbackReturn DiffDriveArduinoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // In the constructor or `on_init` method:
  hw_states_ = {
      {"front_right_wheel_joint", {0.0, 0.0}},  // Position, Velocity
      {"front_left_wheel_joint", {0.0, 0.0}},
      {"rear_right_wheel_joint", {0.0, 0.0}},
      {"rear_left_wheel_joint", {0.0, 0.0}}
  };

  hw_commands_ = {
      {"front_right_wheel_joint", 0.0},  // Velocity
      {"front_left_wheel_joint", 0.0},
      {"rear_right_wheel_joint", 0.0},
      {"rear_left_wheel_joint", 0.0}
  };

  node_ = rclcpp::Node::make_shared("diffdrive_arduino_node");
  command_publisher_ = node_->create_publisher<msg_utils::msg::WheelCommands>("cmd_vel_desired", 10);
  command_feedback_subscriber_ = node_->create_subscription<msg_utils::msg::FourMotorsFeedback>(
        "/cmd_vel_feedback",
        10,
        std::bind(&DiffDriveArduinoHardwareInterface::feedback_callback, this, std::placeholders::_1)
    );


  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto& [key, value] : hw_states_) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(key, "position", &value[0])); // Position
        state_interfaces.emplace_back(hardware_interface::StateInterface(key, "velocity", &value[1])); // Velocity
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto& [key, value] : hw_commands_) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(key, "velocity", &value)); // Velocity
    }

    return command_interfaces;
}

hardware_interface::return_type DiffDriveArduinoHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // La lecture des états est maintenant gérée par le subscriber `/cmd_vel_feedback`
    // Rien à faire ici
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type DiffDriveArduinoHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Publish wheel commands to the ROS2 topic
    msg_utils::msg::WheelCommands cmd_msg;
    cmd_msg.front_left_wheel_speed = hw_commands_["front_left_wheel_joint"];
    cmd_msg.front_right_wheel_speed = hw_commands_["front_right_wheel_joint"];
    cmd_msg.rear_right_wheel_speed = hw_commands_["rear_right_wheel_joint"];
    cmd_msg.rear_left_wheel_speed = hw_commands_["rear_left_wheel_joint"];

    command_publisher_->publish(cmd_msg);

    return hardware_interface::return_type::OK;
}


void DiffDriveArduinoHardwareInterface::feedback_callback(const msg_utils::msg::FourMotorsFeedback::SharedPtr msg) {
    // Mettre à jour les états des moteurs à partir du message reçu
    hw_states_["front_left_wheel_joint"] = {msg->motor_front_left.position, msg->motor_front_left.speed};
    hw_states_["front_right_wheel_joint"] = {msg->motor_front_right.position, msg->motor_front_right.speed};
    hw_states_["rear_left_wheel_joint"] = {msg->motor_rear_left.position, msg->motor_rear_left.speed};
    hw_states_["rear_right_wheel_joint"] = {msg->motor_rear_right.position, msg->motor_rear_right.speed};
}


} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardwareInterface, hardware_interface::SystemInterface)
