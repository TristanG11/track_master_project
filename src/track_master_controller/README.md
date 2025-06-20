# Track Master Controller

`track_master_controller` is a ROS 2 node designed to control a differential drive robot with four DC motors (2 per side). It manages motion commands, PID-based motor control, odometry computation, and joint state publishing, making it suitable for real robot integration or simulation.


## Features

* **Velocity Command Processing**

  * Subscribes to `/cmd_vel` (geometry\_msgs/Twist)
  * Applies acceleration/velocity limits
  * Converts linear/angular velocities to wheel speeds

* **PID Control**

  * Computes motor speeds based on desired vs measured velocities
  * PID gains adjustable at runtime via `/robot_pid` topic

* **Wheel Commands**

  * Publishes computed wheel speeds to `/cmd_vel_desired` (custom `WheelCommands` message)

* **Odometry Computation**

  * Uses encoder feedback from `/cmd_vel_feedback` to estimate position and orientation
  * Publishes nav\_msgs/Odometry on `/odom`

* **Joint State Publishing**

  * Publishes joint positions and velocities on `/joint_states` (sensor\_msgs/JointState)

* **TF Broadcasting**

  * Publishes transform from `odom` frame to `base_footprint`


## Topics

| Topic               | Type                                 | Direction   | Description                        |
| ------------------- | ------------------------------------ | ----------- | ---------------------------------- |
| `/cmd_vel`          | `geometry_msgs/Twist`                | Subscribed  | Target robot velocity              |
| `/robot_pid`        | `msg_utils/msg/RobotPid`             | Subscribed  | PID parameters update              |
| `/cmd_vel_feedback` | `msg_utils/msg/FourMotorsFeedback`   | Subscribed  | Feedback from motor encoders       |
| `/cmd_vel_desired`  | `msg_utils/msg/WheelCommands`        | Published   | Computed wheel speeds              |
| `/odom`             | `nav_msgs/msg/Odometry`              | Published   | Robot pose and velocity            |
| `/joint_states`     | `sensor_msgs/msg/JointState`         | Published   | Joint positions and velocities     |
| `tf`                | `geometry_msgs/msg/TransformStamped` | Broadcasted | TF from `odom` to `base_footprint` |

## Parameters (in YAML or declared in code)

| Parameter Name                 | Type      | Default                            | Description                                |
| ------------------------------ | --------- | ---------------------------------- | ------------------------------------------ |
| `update_rate`                  | double    | `25.0`                             | Rate at which PID and wheel cmd run (Hz)   |
| `publish_rate`                 | double    | `50.0`                             | Odometry and joint state publish rate (Hz) |
| `wheel_separation`             | double    | `0.347`                            | Distance between left and right wheels     |
| `wheel_radius`                 | double    | `0.06`                             | Radius of the wheels (m)                   |
| `left_wheel_names`             | string\[] | `["front_left_wheel_joint", ...]`  | Joint names for left wheels                |
| `right_wheel_names`            | string\[] | `["front_right_wheel_joint", ...]` | Joint names for right wheels               |
| `max_linear_velocity`          | double    | `0.5`                              | Max allowed linear velocity (m/s)          |
| `max_angular_velocity`         | double    | `5.0`                              | Max allowed angular velocity (rad/s)       |
| `pid_left.kp` / `.ki` / `.kd`  | double    | Customizable                       | PID gains for left motors                  |
| `pid_right.kp` / `.ki` / `.kd` | double    | Customizable                       | PID gains for right motors                 |


## Build Instructions

```bash
# From your ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select track_master_controller
source install/setup.bash
```

## Run the Node

```bash
ros2 run track_master_controller track_master_controller_node
```