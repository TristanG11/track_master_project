#include "track_master_localization/kalman_filter.hpp"
using std::placeholders::_1;


KalmanFilter::KalmanFilter(const std::string name) : Node(name)
{
    mean_ = 0.0;
    variance_ = 1000.0;
    imu_angular_z_ = 0.0;
    is_first_odom_ = true;
    last_angular_z_ = 0.0;
    motion_ = 0.0;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/diff_cont/odom",10,std::bind(KalmanFilter::odomCb, this,_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu",10,std::bind(KalmanFilter::imuCb, this,_1));
    odom_filtered_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_filtered",10);

}

void KalmanFilter::odomCb(const nav_msgs::msg::Odometry& msg){
    odom_filtered_ = msg;

    if (is_first_odom_){
        is_first_odom_ = false;
        mean_ = msg.twist.twist.angular.z;
        last_angular_z_ = msg.twist.twist.angular.z;

    }

    // statePrediction();
    // measurementUpdate();
}

void KalmanFilter::imuCb(const sensor_msgs::msg::Imu& msg){
    imu_angular_z_ = msg.angular_velocity.z;
}
