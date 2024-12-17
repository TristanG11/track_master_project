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

  node_ = rclcpp::Node::make_shared("diffdrive_arduino_node");
  command_publisher_ = node_->create_publisher<msg_utils::msg::WheelCommands>("cmd_vel_published", 10);

  // Status relative to battery and motors : 


  battery_status_publisher_ = node_->create_publisher<msg_utils::msg::BatteryStatus>("/battery_status", 10);
  motors_status_publisher_ = node_->create_publisher<msg_utils::msg::FourMotorsStatus>("/motor_status", 10);




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
        // Buffer pour contenir les données entrantes
        boost::asio::streambuf response;
        boost::asio::read_until(*serial_port_, response, "\n"); // Lire jusqu'à un caractère de nouvelle ligne

        // Traitement du flux de données
        std::istream response_stream(&response);
        std::string line;
        std::getline(response_stream, line);

        //RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Raw response received: %s", line.c_str());

        // Vérifiez si le message est une erreur
        if (line.find("Error") == 0) {
            // Affichez le message d'erreur
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Erreur détectée : %s", line.c_str());
            return hardware_interface::return_type::OK;
        }

        msg_utils::msg::BatteryStatus battery_msg;
        msg_utils::msg::FourMotorsStatus motors_msg;

        // Vérifier le type de message global
        if (line.find("status_") != std::string::npos) {
            // Processus pour status_motor_
            
            std::istringstream line_stream(line);
            std::string segment;

            while (std::getline(line_stream, segment, ';')) {
                if (segment.find("status_") == 0) {
                    std::string hw_name = segment.substr(7); // Supprimer "status_" du nom
                    std::istringstream segment_stream(segment);
                    std::string id;

                    if(hw_name.find("battery")!= std::string::npos){
                        
                        double voltage, current, charge_level;
                        int charging;

                        if (std::getline(segment_stream, id, ',') &&
                        segment_stream >> voltage &&
                        segment_stream.ignore(1) &&
                        segment_stream >> current &&
                        segment_stream.ignore(1) &&
                        segment_stream >> charge_level &&
                        segment_stream.ignore(1) &&
                        segment_stream >> charging) {

                        RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), 
             "Battery values parsed: voltage=%f, current=%f, charge_level=%f, charging=%d",
             voltage, current, charge_level, charging);
                                
                        // Publier le message BatteryStatus
                        battery_msg.voltage = voltage;
                        battery_msg.current = current;
                        battery_msg.charge_level = charge_level;
                        battery_msg.charging = (charging ? true : false);   
                    }
                    }else{
                        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), " détectée : %s", line.c_str());
                        double desired_speed, speed, current, voltage;
                        if (std::getline(segment_stream, id, ',') &&
                        segment_stream >> desired_speed &&
                        segment_stream.ignore(1) &&
                        segment_stream >> speed &&
                        segment_stream.ignore(1) &&
                        segment_stream >> current &&
                        segment_stream.ignore(1) &&
                        segment_stream >> voltage) {
                            // Remplir le message motors_msg
                        if (hw_name.find("fl")!=std::string::npos) {
                            motors_msg.motor_front_left.motor_name = "front_left";
                            motors_msg.motor_front_left.desired_speed = desired_speed;
                            motors_msg.motor_front_left.speed = speed;
                            motors_msg.motor_front_left.current = current;
                            motors_msg.motor_front_left.voltage = voltage;
                        } else if (hw_name.find("fr")!=std::string::npos) {
                            motors_msg.motor_front_right.motor_name = "front_right";
                            motors_msg.motor_front_right.desired_speed = desired_speed;
                            motors_msg.motor_front_right.speed = speed;
                            motors_msg.motor_front_right.current = current;
                            motors_msg.motor_front_right.voltage = voltage;
                        } else if (hw_name.find("rl")!=std::string::npos) {
                            motors_msg.motor_rear_left.motor_name = "rear_left";
                            motors_msg.motor_rear_left.desired_speed = desired_speed;
                            motors_msg.motor_rear_left.speed = speed;
                            motors_msg.motor_rear_left.current = current;
                            motors_msg.motor_rear_left.voltage = voltage;
                        } else if (hw_name.find("rr")!=std::string::npos) {
                            motors_msg.motor_rear_right.motor_name = "rear_right";
                            motors_msg.motor_rear_right.desired_speed = desired_speed;
                            motors_msg.motor_rear_right.speed = speed;
                            motors_msg.motor_rear_right.current = current;
                            motors_msg.motor_rear_right.voltage = voltage;
                        }
                    }
                    }
                }
            }
            auto now = node_->now();
            motors_msg.header.stamp = now;
            battery_msg.header.stamp = now;
            battery_status_publisher_->publish(battery_msg);
            motors_status_publisher_->publish(motors_msg);
        } else {
            // Processus pour joint states
            std::istringstream line_stream(line);
            std::string segment;

            while (std::getline(line_stream, segment, ';')) {          
                std::istringstream segment_stream(segment);
                std::string joint_name;
                double position, velocity;

                if (std::getline(segment_stream, joint_name, ',') &&
                    segment_stream >> position &&
                    segment_stream.ignore(1) &&
                    segment_stream >> velocity) {

                    if (hw_states_.find(joint_name) != hw_states_.end()) {
                        hw_states_[joint_name][0] = position; // Mettre à jour la position
                        hw_states_[joint_name][1] = velocity; // Mettre à jour la vitesse
                    } else {
                        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Raw data: %s", line.c_str());
                        RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardwareInterface"), "Unknown joint: %s", joint_name.c_str());  
                    }
                }
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
