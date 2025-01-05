#ifndef NMEA_GNSS_NODE_HPP
#define NMEA_GNSS_NODE_HPP

#include <optional>
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "msg_utils/msg/gps_velocity_heading.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <boost/asio.hpp>
#include "nmea_gnss_cpp/nmea_parser.hpp"
#include <rclcpp/time.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class NmeaGnssNode : public rclcpp::Node {
public:
    // Constructeur et destructeur
    NmeaGnssNode();
    ~NmeaGnssNode();

    // Méthode publique pour lire et traiter le flux GNSS
    void get_gnss_stream();

private:
    // Méthodes utilitaires privées
    void read_serial_data(); // Lire les données du port série
    void parse_and_publish(const std::string &nmea_sentence); // Analyser et publier les données
    void handle_error(const std::string &error_message); // Gérer les erreurs

    // Messages ROS2
    std::shared_ptr<sensor_msgs::msg::NavSatFix> nav_sat_fix_msg_;
    std::shared_ptr<msg_utils::msg::GpsVelocityHeading> gps_velocity_heading_msg_;

    // Éditeurs ROS2
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
    rclcpp::Publisher<msg_utils::msg::GpsVelocityHeading>::SharedPtr gps_velocity_heading_publisher_;

    // Timer pour exécuter périodiquement une tâche
    rclcpp::TimerBase::SharedPtr timer_;

    // Gestion de la communication série avec Boost.Asio
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;

    // Paramètres configurables
    std::string serial_port_name_; // Nom du port série
    int baud_rate_;                // Débit en bauds pour la communication

    // Variables de diagnostic
    bool serial_port_open_ = false;  // État du port série
    std::optional<rclcpp::Time> last_valid_gga_time_;  // Dernier GGA valide
    std::optional<rclcpp::Time> last_valid_vtg_time_;  // Dernier VTG valide

    // Diagnostic updater
    diagnostic_updater::Updater diagnostics_updater_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

};

#endif // NMEA_GNSS_NODE_HPP
