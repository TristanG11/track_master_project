#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import math
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from msg_utils.msg import GpsVelocityHeading

# Define an anchor point: lat 45.1884999, lon 5.7588211
anchor_lat = 45.1884999
anchor_lon = 5.7588211


class MapFramePublisher(Node):
    def __init__(self):
        super().__init__('map_frame_publisher')

        # Initialize parameters and variables
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.heading = 0.0  # Initialize heading

        # Subscriptions
        self.gnss_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, 10)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        if self.use_sim_time:
            self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_cb, qos)
        else:
            self.gnss_vel_heading_sub = self.create_subscription(
                GpsVelocityHeading, "/gnss/heading_vel", self.gnss_vel_heading_cb, 10
            )

        # Variables
        self.first_fix = None
        self.first_fix_fetched = False
        self.map_frame_computed = False

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.publish_transform)
        self.map_to_odom = TransformStamped()

    def gps_cb(self, msg):
        """
        Callback for GPS data. Stores the first GPS fix when valid data is received.
        """
        if not self.first_fix_fetched:
            if msg.status.service > 0:  # Ensure we have a valid GNSS fix
                self.first_fix = msg
                self.first_fix_fetched = True
                self.get_logger().info(f"First GNSS Fix: {msg.latitude}, {msg.longitude}, {msg.altitude}")
                self.compute_map_frame()
                self.destroy_subscription(self.gnss_sub)

    def gnss_vel_heading_cb(self, msg):
        """
        Callback for GNSS heading and velocity.
        """
        if msg.valid.data:
            self.heading = msg.heading.data
            self.get_logger().info(f"Updated heading: {self.heading:.2f}°")
        else:
            self.get_logger().warning("Invalid GNSS heading/velocity data received.")

    def clock_cb(self, msg):
        """
        Callback for /clock topic to update the timestamp in the map_to_odom transform.
        """
        if self.map_frame_computed:
            self.map_to_odom.header.stamp.sec = msg.clock.sec
            self.map_to_odom.header.stamp.nanosec = msg.clock.nanosec

    def compute_map_frame(self):
        """
        Computes the map->odom transform using ENU coordinates relative to the anchor point.
        """
        if self.first_fix:
            e, n, u = pm.geodetic2enu(
                anchor_lat,
                anchor_lon,
                self.first_fix.altitude,  # Use 0 altitude for anchor point
                self.first_fix.latitude,
                self.first_fix.longitude,
                self.first_fix.altitude,
                pm.Ellipsoid.from_name('wgs84'),
            )

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = e
            t.transform.translation.y = n
            t.transform.translation.z = u

            # Set rotation (heading)
            quat = quaternion_from_euler(0, 0, self.heading * math.pi / 180.0)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.map_to_odom = t
            self.map_frame_computed = True

    def publish_transform(self):
        """
        Publishes the map->odom transform.
        """
        if self.map_frame_computed:
            if not self.use_sim_time:
                self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.map_to_odom)


def main():
    rclpy.init(args=None)
    node = MapFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user. Shutting down.")
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
