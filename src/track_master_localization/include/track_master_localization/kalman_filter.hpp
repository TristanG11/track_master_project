#ifndef KALMAN_FILTER.HPP
#define KALMAN_FILTER.HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node {
public :
    KalmanFilter(const std::string name);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pub_;

    double mean_;
    double variance_;
    double imu_angular_z_;
    bool is_first_odom_ = true;
    double last_angular_z_;
    double motion_;

    nav_msgs::msg::Odometry odom_filtered_;

    void odomCb(const nav_msgs::msg::Odometry& msg);

    void imuCb(const sensor_msgs::msg::Imu& msg);
};


#endif