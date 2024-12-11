#include "nmea_gnss_cpp/nmea_gnss.hpp"
#include <iostream>
#include <sstream>

NmeaGnssNode::NmeaGnssNode()
    : Node("nmea_gnss_node"), serial_port_name_("/dev/ttyUSB0"), baud_rate_(38400) {
    // Déclaration des éditeurs
    nav_sat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gnss/fix", 10);
    gps_velocity_heading_publisher_ = this->create_publisher<msg_utils::msg::GpsVelocityHeading>("/gnss/heading_vel", 10);

    // Initialisation de Boost.Asio pour la communication série
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_);
        serial_port_->open(serial_port_name_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        throw;
    }

    // Création d'un timer pour lire le flux GNSS périodiquement
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // Période de 200 ms
        std::bind(&NmeaGnssNode::get_gnss_stream, this));
}

NmeaGnssNode::~NmeaGnssNode() {
    // Fermeture du port série proprement
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
}

void NmeaGnssNode::get_gnss_stream() {
    try {
        read_serial_data();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading GNSS stream: %s", e.what());
    }
}

void NmeaGnssNode::read_serial_data() {
    if (!serial_port_ || !serial_port_->is_open()) {
        throw std::runtime_error("Serial port not open");
    }

    char buffer[512];
    boost::system::error_code error;
    size_t bytes_read = serial_port_->read_some(boost::asio::buffer(buffer), error);

    if (error && error != boost::asio::error::eof) {
        throw std::runtime_error("Error reading from serial port: " + error.message());
    }

    // Traiter les données reçues comme une chaîne de caractères
    std::string data(buffer, bytes_read);
    std::istringstream stream(data);
    std::string nmea_sentence;

    while (std::getline(stream, nmea_sentence)) {
        parse_and_publish(nmea_sentence);
    }
}

void NmeaGnssNode::parse_and_publish(const std::string &nmea_sentence) {
    NmeaParser parser;

    try {
        if (nmea_sentence.find("GGA") != std::string::npos) {
            // Analyse de la phrase GGA
            parser.parse_gga(nmea_sentence);
            

            // Récupération des données
            double latitude = parser.get_lat((*parser.gga_msg)["latitude"]);
            double longitude = parser.get_lon((*parser.gga_msg)["longitude"]);
            int service = parser.get_service_flags((*parser.gga_msg)["service"]);
            int status = parser.get_status_flags((*parser.gga_msg)["fix_quality"]);
            double altitude = std::stod((*parser.gga_msg)["altitude"]);

            // Construction et publication du message ROS2
            nav_sat_fix_msg_ = std::make_shared<sensor_msgs::msg::NavSatFix>();
            nav_sat_fix_msg_->header.stamp = this->now();
            nav_sat_fix_msg_->latitude = latitude;
            nav_sat_fix_msg_->longitude = longitude;
            nav_sat_fix_msg_->altitude = altitude;
            nav_sat_fix_msg_->status.status = status;
            nav_sat_fix_msg_->status.service = service;

            nav_sat_fix_publisher_->publish(*nav_sat_fix_msg_);
            RCLCPP_INFO(this->get_logger(), "Published NavSatFix message");
        } else if (nmea_sentence.find("VTG") != std::string::npos) {
            // Analyse de la phrase VTG
            parser.parse_vtg(nmea_sentence);

            // Récupération des données
            double speed_knots = std::stod((*parser.vtg_msg)["speed_knots"]);
            double speed_kmh = std::stod((*parser.vtg_msg)["speed_kmh"]);

            // Construction et publication du message ROS2
            gps_velocity_heading_msg_ = std::make_shared<msg_utils::msg::GpsVelocityHeading>();
            gps_velocity_heading_msg_->velocity.data = speed_kmh / 3.6;  // Conversion en m/s
            gps_velocity_heading_msg_->heading.data = std::stod((*parser.vtg_msg)["magnetic_course"]);

            gps_velocity_heading_publisher_->publish(*gps_velocity_heading_msg_);
            RCLCPP_INFO(this->get_logger(), "Published GpsVelocityHeading message");
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown NMEA sentence: %s", nmea_sentence.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse NMEA sentence: %s", e.what());
    }
}
