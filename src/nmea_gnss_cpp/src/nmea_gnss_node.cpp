#include "nmea_gnss_cpp/nmea_gnss.hpp"





int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<NmeaGnssNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}