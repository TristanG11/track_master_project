# nmea\_gnss\_rs

A Rust-based ROS 2 node for parsing and publishing GNSS data from an NMEA-compatible receiver via serial interface.

## Features

* Connects to a GNSS receiver over serial (default: `/dev/gnss`)
* Parses NMEA sentences (GGA and VTG) to extract position, velocity, and heading
* Converts geodetic coordinates to ENU using `map_3d`
* Publishes:

  * `/gnss/fix` (`sensor_msgs/NavSatFix`)
  * `/gnss/pose` (`geometry_msgs/PoseWithCovarianceStamped`)
  * `/gnss/twist` (`geometry_msgs/TwistWithCovarianceStamped`)
  * `/gnss/heading_vel` (`msg_utils/GpsVelocityHeading`)
* Publishes diagnostics to `/diagnostics`
* Automatic serial reconnection and error handling

## Parameters

| Name                | Type     | Default             | Description                                 |
| ------------------- | -------- | ------------------- | ------------------------------------------- |
| `baud_rate`         | `int`    | `38400`             | Serial communication baud rate              |
| `lat0`              | `double` | `45.1884999`        | Reference latitude for ENU transformation   |
| `lon0`              | `double` | `5.7588211`         | Reference longitude                         |
| `alt0`              | `double` | `0.0`               | Reference altitude                          |
| `gnss_fix_topic`    | `string` | `/gnss/fix`         | Output topic for GPS position               |
| `heading_vel_topic` | `string` | `/gnss/heading_vel` | Topic for GNSS heading and velocity         |
| `twist_topic`       | `string` | `/gnss/twist`       | Topic for linear velocity in `Twist` format |
| `pose_topic`        | `string` | `/gnss/pose`        | Topic for pose in ENU frame                 |

## Usage

Build and run the node with ROS 2:

```bash
ros2 run nmea_gnss_rs nmea_gnss_node
```

You can override parameters using a launch file or YAML configuration.

## Dependencies

* ROS 2 (`rclrs`)
* `serialport`, `nmea-parser`, `map_3d`
* Custom message package: `msg_utils`

## Example Applications

* Outdoor mobile robotics
* GNSS-based pose estimation pipelines
* Sensor fusion with odometry or IMU
