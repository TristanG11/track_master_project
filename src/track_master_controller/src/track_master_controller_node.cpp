

// track_master_controller.cpp
#include "track_master_controller/track_master_controller_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

/// Ramp between current and target by at most a_max * dt
double ramp(double &current, double &target, double &a_max, double dt) {
    double diff = target - current;
    double max_step = a_max * dt;
    if (diff >  max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
  }

double smooth(double current, double target, double alpha) {
    return current + alpha * (target - current);
}

namespace track_master_controller
{

TrackMasterController::TrackMasterController(const rclcpp::NodeOptions & options)
: Node("track_master_controller", options)
{
  if (!initParams()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10,
    std::bind(&TrackMasterController::cmdVelCallback, this, std::placeholders::_1));

  feedback_sub_ = create_subscription<msg_utils::msg::FourMotorsFeedback>(
    feedback_topic_, 10,
    std::bind(&TrackMasterController::feedbackCallback, this, std::placeholders::_1));

  robot_pid_sub_ = create_subscription<msg_utils::msg::RobotPid>(
    robot_pid_topic_,10,
    std::bind(&TrackMasterController::robotPidCallback, this, std::placeholders::_1));

  // publishers
  wheel_cmd_pub_ = create_publisher<msg_utils::msg::WheelCommands>(wheel_cmd_topic_, 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  // timers
  odom_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&TrackMasterController::publishOdometry, this));

  joint_state_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&TrackMasterController::publishJointStates, this));

  pid_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),  // 20 Hz
  std::bind(&TrackMasterController::pidControlCallback, this)
  );

    joint_state_msg_.name = {
        left_wheel_names_[0],
        left_wheel_names_[1],
        right_wheel_names_[0],
        right_wheel_names_[1]
      };

  wheel_cmd_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&TrackMasterController::publishWheelCommands, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  last_time_ = this->now();
}

bool TrackMasterController::initParams()
{
  declare_parameter<double>("update_rate", 25.0);
  declare_parameter<double>("publish_rate", 50.0);
  declare_parameter<std::string>("odom_frame_id", "odom");
  declare_parameter<std::string>("base_frame_id", "base_footprint");
  declare_parameter<double>("wheel_separation", 0.347);
  declare_parameter<double>("wheel_radius", 0.06);

  declare_parameter<std::vector<std::string>>("left_wheel_names", {"front_left_wheel_joint", "rear_left_wheel_joint"});
  declare_parameter<std::vector<std::string>>("right_wheel_names", {"front_right_wheel_joint", "rear_right_wheel_joint"});

  declare_parameter<std::string>("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped");
  declare_parameter<std::string>("wheel_cmd_topic", "/cmd_vel_desired");
  declare_parameter<std::string>("feedback_topic", "/cmd_vel_feedback");
  declare_parameter<std::string>("odom_topic", "/odom");
  declare_parameter<std::string>("joint_state_topic", "/joint_states");
  declare_parameter<std::string>("robot_pid_topic","/robot_pid");

  // Déclaration des limites maximales
  declare_parameter<double>("max_linear_velocity", 0.5);
  declare_parameter<double>("max_linear_acceleration", 1.0); 
  declare_parameter<double>("max_angular_velocity", 5.0); 
  declare_parameter<double>("max_angular_acceleration", 1.0);  

  // Déclaration des limites minimales
  declare_parameter<double>("min_linear_velocity", 0.0); 
  declare_parameter<double>("min_linear_acceleration", 0.0);  
  declare_parameter<double>("min_angular_velocity", 0.0);  
  declare_parameter<double>("min_angular_acceleration", 0.0);  

  declare_parameter<double>("pid_left.kp", 1.0);
  declare_parameter<double>("pid_left.ki", 0.0);
  declare_parameter<double>("pid_left.kd", 0.1);
  
  declare_parameter<double>("pid_right.kp", 1.0);
  declare_parameter<double>("pid_right.ki", 0.0);
  declare_parameter<double>("pid_right.kd", 0.1);


  update_rate_ = get_parameter("update_rate").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  wheel_separation_ = get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_parameter("wheel_radius").as_double();

  left_wheel_names_ = get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = get_parameter("right_wheel_names").as_string_array();

  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  wheel_cmd_topic_ = get_parameter("wheel_cmd_topic").as_string();
  feedback_topic_ = get_parameter("feedback_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  robot_pid_topic_ = get_parameter("robot_pid_topic").as_string();
  // Récupération des limites maximales
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_linear_acceleration_ = get_parameter("max_linear_acceleration").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
  max_angular_acceleration_ = get_parameter("max_angular_acceleration").as_double();

  // Récupération des limites minimales
  min_linear_velocity_ = get_parameter("min_linear_velocity").as_double();
  min_linear_acceleration_ = get_parameter("min_linear_acceleration").as_double();
  min_angular_velocity_ = get_parameter("min_angular_velocity").as_double();
  min_angular_acceleration_ = get_parameter("min_angular_acceleration").as_double();


  pid_left_.kp_ = get_parameter("pid_left.kp").as_double();
  pid_left_.ki_ = get_parameter("pid_left.ki").as_double();
  pid_left_.kd_ = get_parameter("pid_left.kd").as_double();

  pid_right_.kp_ = get_parameter("pid_right.kp").as_double();
  pid_right_.ki_ = get_parameter("pid_right.ki").as_double();
  pid_right_.kd_ = get_parameter("pid_right.kd").as_double();



  return true;
}

void TrackMasterController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vel_linear_desired_  = msg->linear.x;
  vel_angular_desired_ = msg->angular.z; 

  const double dt = 1.0 / 50.0;

  v_est_     = ramp(v_est_,     vel_linear_desired_,  max_linear_acceleration_,  dt);
  omega_est_ = ramp(omega_est_, vel_angular_desired_, max_angular_acceleration_, dt);

  v_est_     = std::clamp(v_est_,     min_linear_velocity_,  max_linear_velocity_);
  omega_est_ = std::clamp(omega_est_, min_angular_velocity_, max_angular_velocity_);

  vl_desired_ = v_est_ - omega_est_ * wheel_separation_ / 2.0; // m/s
  vr_desired_ = v_est_ + omega_est_ * wheel_separation_ / 2.0;  // m/S
}

void TrackMasterController::feedbackCallback(const msg_utils::msg::FourMotorsFeedback::SharedPtr msg)
{
  last_feedback_ = *msg;

  // Compute velocity 
  vl_measured_ = (last_feedback_.motor_front_left.speed + last_feedback_.motor_rear_left.speed) / 2.0 * wheel_radius_; // m/s
  vr_measured_ = (last_feedback_.motor_front_right.speed + last_feedback_.motor_rear_right.speed) / 2.0 * wheel_radius_;// m/s
  v_measured_ = (vl_measured_ + vr_measured_) / 2.0;
  omega_measured_ = (vr_measured_ - vl_measured_) / wheel_separation_;

  // Compute position 
  double dl = (last_feedback_.motor_front_left.position + last_feedback_.motor_rear_left.position) / 2.0;
  double dr = (last_feedback_.motor_front_right.position + last_feedback_.motor_rear_right.position) / 2.0;

  delta_dl_ = dl - prev_dl_;
  delta_dr_ = dr - prev_dr_;

  prev_dl_ = dl;
  prev_dr_ = dr;

  delta_s_ = (delta_dr_ + delta_dl_) / 2.0;
  delta_theta_ = (delta_dr_ - delta_dl_) / wheel_separation_;
  theta_ += delta_theta_;

  odom_msg_.header.stamp = get_clock()->now();
  odom_msg_.header.frame_id = odom_frame_id_;
  odom_msg_.child_frame_id = base_frame_id_;

  odom_msg_.twist.twist.linear.x = v_measured_;
  odom_msg_.twist.twist.angular.z = omega_measured_;


  odom_msg_.pose.pose.position.x += delta_s_ * cos(theta_ + delta_theta_ / 2.0);
  odom_msg_.pose.pose.position.y += delta_s_ * sin(theta_ + delta_theta_ / 2.0);

  tf2::Quaternion quat;
  quat.setRPY( 0, 0, theta_ );
  odom_msg_.pose.pose.orientation = tf2::toMsg(quat);
}

void TrackMasterController::publishOdometry()
{


  odom_pub_->publish(odom_msg_);

  // Publish TF
  odom_tf_.header.stamp = this->get_clock()->now();
  odom_tf_.header.frame_id = odom_frame_id_;
  odom_tf_.child_frame_id = base_frame_id_;

  odom_tf_.transform.translation.x = odom_msg_.pose.pose.position.x;
  odom_tf_.transform.translation.y = odom_msg_.pose.pose.position.y;
  odom_tf_.transform.translation.z = odom_msg_.pose.pose.position.z;
  odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;

  tf_broadcaster_->sendTransform(odom_tf_);

}

void TrackMasterController::publishJointStates()
{
  joint_state_msg_.header.stamp = get_clock()->now();

  // On remplace entièrement le contenu du vector en une seule opération
  
  joint_state_msg_.position = {
    last_feedback_.motor_front_left.position,
    last_feedback_.motor_rear_left.position,
    last_feedback_.motor_front_right.position,
    last_feedback_.motor_rear_right.position
  };

  joint_state_msg_.velocity = {
    last_feedback_.motor_front_left.speed,
    last_feedback_.motor_rear_left.speed,
    last_feedback_.motor_front_right.speed,
    last_feedback_.motor_rear_right.speed
  };

  joint_state_pub_->publish(joint_state_msg_);
}


void TrackMasterController::publishWheelCommands()
{

  msg_utils::msg::WheelCommands cmd;
  cmd.front_left_wheel_speed = wl_desired_;
  cmd.rear_left_wheel_speed = wl_desired_;
  cmd.front_right_wheel_speed = wr_desired_;
  cmd.rear_right_wheel_speed = wr_desired_;

  wheel_cmd_pub_->publish(cmd);
}

void TrackMasterController::robotPidCallback(const msg_utils::msg::RobotPid::SharedPtr msg)
{
  pid_left_.kp_ = msg->pid_left.kp;
  pid_left_.ki_ = msg->pid_left.ki;
  pid_left_.kd_ = msg->pid_left.kd;

  pid_right_.kp_ = msg->pid_right.kp;
  pid_right_.ki_ = msg->pid_right.ki;
  pid_right_.kd_ = msg->pid_right.kd;

}
void TrackMasterController::pidControlCallback()
{
  rclcpp::Time now = this->now();
  double dt = (now - last_time_).seconds();
  if (dt <= 0.0) return;
  last_time_ = now;
  
  // Calcul des erreurs

  double error_left = vl_desired_ - vl_measured_;
  double error_right = vr_desired_ - vr_measured_;

  // Commandes PID => calcul des vitesses angulaires pour chaque moteur
  vl_corrected_ = vl_desired_;//pid_left_.compute(error_left, dt);
  vr_corrected_ = vr_desired_;//pid_right_.compute(error_right, dt);

  wl_desired_ = vl_corrected_ / wheel_radius_;
  wr_desired_ = vr_corrected_ / wheel_radius_;

}

} // namespace track_master_controller
