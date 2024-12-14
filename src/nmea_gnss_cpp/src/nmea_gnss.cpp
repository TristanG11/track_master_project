#include "nmea_gnss_cpp/nmea_gnss.hpp"
#include <iostream>
#include <sstream>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

NmeaGnssNode::NmeaGnssNode()
    : Node("nmea_gnss_node"),diagnostics_updater_(this, 0.5){
    // Déclaration des éditeurs
    this->declare_parameter<std::string>("serial_port_name", "/dev/ttyUSB3");
    this->declare_parameter<int>("baud_rate", 38400);
    this->declare_parameter<std::string>("topic_nav_sat_fix", "/gnss/fix");
    this->declare_parameter<std::string>("topic_gps_velocity_heading", "/gnss/heading_vel");

    // Lire les paramètres (ceux du fichier YAML seront utilisés si chargés via launch)
    serial_port_name_ = this->get_parameter("serial_port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    std::string topic_nav_sat_fix = this->get_parameter("topic_nav_sat_fix").as_string();
    std::string topic_gps_velocity_heading = this->get_parameter("topic_gps_velocity_heading").as_string();

    // Déclaration des éditeurs avec les noms des topics configurés
    nav_sat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_nav_sat_fix, 10);
    gps_velocity_heading_publisher_ = this->create_publisher<msg_utils::msg::GpsVelocityHeading>(topic_gps_velocity_heading, 10);

    // , serial_port_name_("/dev/ttyUSB0"), baud_rate_(38400) 
    // Initialisation de Boost.Asio pour la communication série
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_);
        serial_port_open_ = true;
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

    // Initialisation des diagnostics
    diagnostics_updater_.setHardwareID("NMEA GNSS Node");

    // Diagnostic pour le port série
    diagnostics_updater_.add("Serial Port Status", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (serial_port_open_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial port is open");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port is closed");
        }
    });

    // Diagnostic pour les messages valides
    diagnostics_updater_.add("Last Valid Messages", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
        rclcpp::Time now = this->now();

        // Temps écoulé depuis le dernier message GGA valide
        if (last_valid_gga_time_) {
            double gga_delta = (now - *last_valid_gga_time_).seconds();
            stat.add("Time since last valid GGA (seconds)", gga_delta);
        } else {
            stat.add("Time since last valid GGA (seconds)", "No valid GGA received");
        }

        // Temps écoulé depuis le dernier message VTG valide
        if (last_valid_vtg_time_) {
            double vtg_delta = (now - *last_valid_vtg_time_).seconds();
            stat.add("Time since last valid VTG (seconds)", vtg_delta);
        } else {
            stat.add("Time since last valid VTG (seconds)", "No valid VTG received");
        }

        // Résumé
        if (last_valid_gga_time_ && last_valid_vtg_time_ &&
            (now - *last_valid_gga_time_).seconds() < 10.0 &&
            (now - *last_valid_vtg_time_).seconds() < 10.0) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Receiving valid GGA and VTG messages");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Delayed GGA or VTG messages");
        }
    });

    // Initialisation : aucune donnée valide reçue
    last_valid_gga_time_ = std::nullopt;
    last_valid_vtg_time_ = std::nullopt;

}

NmeaGnssNode::~NmeaGnssNode() {
    // Fermeture du port série proprement
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
    diagnostics_timer_.reset();
    timer_.reset();
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
            // Analyse et publication du message GGA
            last_valid_gga_time_ = this->now();  // Mettre à jour l'horodatage GGA
            RCLCPP_INFO(this->get_logger(), "Valid GGA message received");
        } else if (nmea_sentence.find("VTG") != std::string::npos) {
            // Analyse de la phrase VTG
            parser.parse_vtg(nmea_sentence);


            // Récupération des données
            double speed_knots = std::stod((*parser.vtg_msg)["speed_knots"]);
            double speed_kmh = std::stod((*parser.vtg_msg)["speed_kmh"]);

            // Construction et publication du message ROS2
            gps_velocity_heading_msg_ = std::make_shared<msg_utils::msg::GpsVelocityHeading>();
            gps_velocity_heading_msg_->velocity = speed_kmh / 3.6;  // Conversion en m/s
            gps_velocity_heading_msg_->heading = std::stod((*parser.vtg_msg)["magnetic_course"]);

            gps_velocity_heading_publisher_->publish(*gps_velocity_heading_msg_);
            // Analyse et publication du message VTG
            last_valid_vtg_time_ = this->now();  // Mettre à jour l'horodatage VTG
            RCLCPP_INFO(this->get_logger(), "Valid VTG message received");
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown NMEA sentence: %s", nmea_sentence.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse NMEA sentence: %s", e.what());
    }
}



