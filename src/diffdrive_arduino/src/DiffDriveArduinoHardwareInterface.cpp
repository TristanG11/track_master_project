#include "diffdrive_arduino/DiffDriveArduinoHardwareInterface.hpp"

namespace diffdrive_arduino
{

DiffDriveArduinoHardwareInterface::DiffDriveArduinoHardwareInterface()
  : serial_port_(std::make_unique<boost::asio::serial_port>(io_service_)) {}  

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

  command_publisher_ = rclcpp::Node::make_shared("diffdrive_arduino_node")->create_publisher<msg_utils::msg::WheelCommands>("cmd_vel_published", 10);
  //state_publisher_ = rclcpp::Node::make_shared("diffdrive_arduino_node")->create_publisher<geometry_msgs::msg::Twist>("wheel_states_published", 10);

  // Open Serial port
  try {
    serial_port_->open("/dev/ttyACM0");
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  } catch (boost::system::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Erreur lors de l'ouverture du port série: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

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
    try {
        // Buffer to hold the incoming data
        boost::asio::streambuf response;
        boost::asio::read_until(*serial_port_, response, "\n"); // Read until newline character

        // Stream to process the data
        std::istream response_stream(&response);
        std::string line;
        std::getline(response_stream, line);

        // Log the raw response
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Raw response received: %s", line.c_str());

        // Example incoming data: "front_left_wheel_joint,0.12,0.34;front_right_wheel_joint,0.11,0.33;..."

        // Parse the response
        std::istringstream line_stream(line);
        std::string segment;

        while (std::getline(line_stream, segment, ';')) {
            // Example segment: "front_left_wheel_joint,0.12,0.34"
            std::istringstream segment_stream(segment);
            std::string joint_name;
            double position, velocity;

            // Extract the joint name and values
            if (std::getline(segment_stream, joint_name, ',') &&
                segment_stream >> position &&
                segment_stream.ignore(1) && // Ignore the comma
                segment_stream >> velocity) {
                
                // Update the hardware state if the joint exists
                if (hw_states_.find(joint_name) != hw_states_.end()) {
                    hw_states_[joint_name][0] = position; // Update position
                    hw_states_[joint_name][1] = velocity; // Update velocity
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Unknown joint: %s", joint_name.c_str());
                }
            } 
            else if (!(std::getline(segment_stream, joint_name, ','))){}
            
            else {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Malformed segment: %s", segment.c_str());
                //return hardware_interface::return_type::ERROR;
            }
        }
    } catch (boost::system::system_error &e) {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Read error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}



hardware_interface::return_type DiffDriveArduinoHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Construct the command string
    std::ostringstream command;
    for (const auto& [key, value] : hw_commands_) {
        command << key << "," << value << ";";
    }
    command << "\n";

    try {
        // Write the command string to Arduino
        boost::asio::write(*serial_port_, boost::asio::buffer(command.str()));
    } catch (boost::system::system_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Write error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    // Publish wheel commands to the ROS2 topic
    msg_utils::msg::WheelCommands cmd_msg;
    cmd_msg.front_left_wheel_speed = hw_commands_["front_left_wheel_joint"];
    cmd_msg.front_right_wheel_speed = hw_commands_["front_right_wheel_joint"];
    cmd_msg.rear_right_wheel_speed = hw_commands_["rear_right_wheel_joint"];
    cmd_msg.rear_left_wheel_speed = hw_commands_["rear_left_wheel_joint"];

    command_publisher_->publish(cmd_msg);

    return hardware_interface::return_type::OK;
}




} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardwareInterface, hardware_interface::SystemInterface)
