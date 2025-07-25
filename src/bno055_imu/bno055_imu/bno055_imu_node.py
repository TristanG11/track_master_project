import time
import board
import busio
import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        # Appliquer les offsets de calibration
        self.sensor.offsets_accelerometer = (-34, -15, -18)
        self.sensor.offsets_gyroscope = (-1, 0, -1)
        self.sensor.offsets_magnetometer = (293, -1039, -463)
        self.sensor.radius_accelerometer = 1000
        self.sensor.radius_magnetometer = 777
        # Mode NDOF (fusion absolue)
        self.sensor.mode = adafruit_bno055.NDOF_MODE
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation (quaternion)
        quat = self.sensor.quaternion
        if quat is not None:
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        else:
            imu_msg.orientation_covariance = [-1.0]*9

        # Angular velocity (rad/s)
        ang_vel = self.sensor.gyro
        if ang_vel is not None:
            imu_msg.angular_velocity.x = ang_vel[0]
            imu_msg.angular_velocity.y = ang_vel[1]
            imu_msg.angular_velocity.z = ang_vel[2]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        else:
            imu_msg.angular_velocity_covariance = [-1.0]*9

        # Linear acceleration (m/s^2)
        lin_accel = self.sensor.linear_acceleration
        if lin_accel is not None:
            imu_msg.linear_acceleration.x = lin_accel[0]
            imu_msg.linear_acceleration.y = lin_accel[1]
            imu_msg.linear_acceleration.z = lin_accel[2]
            imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        else:
            imu_msg.linear_acceleration_covariance = [-1.0]*9

        self.imu_pub.publish(imu_msg)

        # Température
        temp = self.sensor.temperature
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = "imu_link"
        temp_msg.temperature = float(temp) if temp is not None else float('nan')
        temp_msg.variance = 0.5
        self.temp_pub.publish(temp_msg)

def main():
    rclpy.init()
    node = BNO055Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()