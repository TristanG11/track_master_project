// track_master_controller_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "msg_utils/msg/wheel_commands.hpp"
#include "msg_utils/msg/four_motors_feedback.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "msg_utils/msg/robot_pid.hpp"
#include <string>
#include <vector>
#include <algorithm>

struct PID {
  double kp_, ki_, kd_;
  double prev_error_ = 0.0;
  double integral_ = 0.0;
  double max_integral_ = 5.0;

  double compute(double error, double dt) {
    // Intégration avec anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);  // correction de l’ordre

    // Dérivée
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    // Sortie PID
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

};


namespace track_master_controller
{

class TrackMasterController : public rclcpp::Node
{
public:
  explicit TrackMasterController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool initParams();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void feedbackCallback(const msg_utils::msg::FourMotorsFeedback::SharedPtr msg);
  void robotPidCallback(const msg_utils::msg::RobotPid::SharedPtr msg);
  
  // Timer callbacks
  void publishOdometry();
  void publishJointStates();
  void publishWheelCommands();
  void pidControlCallback();

  // Parameters
  double update_rate_;
  double publish_rate_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double wheel_separation_;
  double wheel_radius_;

  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;

  std::string cmd_vel_topic_;
  std::string feedback_topic_;
  std::string wheel_cmd_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string robot_pid_topic_;

  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;

  // Limits (optional use)
  double max_linear_velocity_;
  double min_linear_velocity_;
  double max_linear_acceleration_;
  double min_linear_acceleration_;
  double max_angular_velocity_;
  double min_angular_velocity_;
  double max_angular_acceleration_;
  double min_angular_acceleration_;



  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<msg_utils::msg::FourMotorsFeedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<msg_utils::msg::RobotPid>::SharedPtr robot_pid_sub_;
  // Publishers
  rclcpp::Publisher<msg_utils::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  // State
  rclcpp::Time last_time_;

  // Msgs
  sensor_msgs::msg::JointState joint_state_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  msg_utils::msg::FourMotorsFeedback last_feedback_;
  geometry_msgs::msg::TransformStamped odom_tf_;
  double prev_dl_ = 0.0;
  double prev_dr_ = 0.0;

  double v_est_ = 0.0;      
  double omega_est_ = 0.0;

  double vel_linear_desired_ = 0.0;
  double vel_angular_desired_ = 0.0;
  

  double vl_desired_ = 0.0;
  double vr_desired_ = 0.0;
  double vl_measured_ = 0.0;
  double vr_measured_ = 0.0;
  double vl_corrected_ = 0.0;
  double vr_corrected_ = 0.0;
  double v_measured_ = 0.0;
  double omega_measured_ = 0.0;
  double delta_dl_ = 0.0;
  double delta_dr_ = 0.0;
  double delta_s_ = 0.0;
  double delta_theta_ = 0.0;
  double theta_ = 0.0;
  double wl_desired_ = 0.0;
  double wr_desired_ = 0.0;

  double correction_factor_ = 1.0;
  // Timers
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr wheel_cmd_timer_;
  rclcpp::TimerBase::SharedPtr tf_pub_timer_;
  rclcpp::TimerBase::SharedPtr pid_timer_;

  // PID 

  PID pid_left_;
  PID pid_right_;
};

} // namespace track_master_controller
