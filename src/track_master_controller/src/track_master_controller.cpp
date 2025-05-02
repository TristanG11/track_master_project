// track_master_controller_node.cpp

#include "track_master_controller/track_master_controller_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<track_master_controller::TrackMasterController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
