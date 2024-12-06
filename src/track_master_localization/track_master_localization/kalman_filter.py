#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub =self.create_subscription(Odometry,"/diff_drive_controller/odom",self.odom_cb,10)
        self.imu_sub = self.create_subscription(Imu,"imu",self.imu_cb,10)
        self.odom_filtered_pub = self.create_publisher(Odometry,"odom_filtered",10)

        #use of a gaussian distribution with huge uncertainty
        self.mean = 0.0
        self.variance = 1.0 

        # We are going to fuse only robot angular vel
        self.imu_angular_z = 0.0 
        self.is_first_odom = True
        self.last_angular_z = 0.0 

        self.motion_variance = 0.05 #relative to odometry
        self.measurement_variance = 5.0 #relative to imu

        self.motion = 0.0 #difference betwee two consecutive angular velocities
        self.filtered_odom = Odometry()

    def imu_cb(self, msg):
        self.imu_angular_z = msg.angular_velocity.z

    def odom_cb(self, msg):
        self.filtered_odom = msg  # first estimate

        if self.is_first_odom :
            self.is_first_odom = False

            self.mean = msg.twist.twist.angular.z
            self.last_angular_z = msg.twist.twist.angular.z
            return
        
        self.motion = msg.twist.twist.angular.z - self.last_angular_z
        print(self.last_angular_z)
        self.state_prediction()

        self.measurement_update()

        self.filtered_odom.twist.twist.angular.z = self.mean
        self.odom_filtered_pub.publish(self.filtered_odom)
        self.last_angular_z = msg.twist.twist.angular.z

    def measurement_update(self):
        self.mean = (self.measurement_variance * self.mean + self.variance * self.imu_angular_z ) / (self.variance + self.measurement_variance)
        self.variance = (self.variance * self.measurement_variance) / (self.variance + self.measurement_variance)

    def state_prediction(self):
        self.mean += self.motion
        self.variance += self.motion_variance

        
if __name__ == '__main__':
    rclpy.init()
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()