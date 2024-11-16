import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
import time
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus  # Added import

class JoystickControlNode(Node):
    # Constants for joystick control
    AXIS_LINEAR = 4  # Axis for linear speed
    AXIS_ANGULAR = 3  # Axis for angular speed
    BUTTON_STOP = 0  # Button to stop the robot
    BUTTON_START = 2  # Button to resume robot control
    DEADZONE_THRESHOLD = 0.05  # Threshold for joystick dead zone

    def __init__(self):
        super().__init__('joystick_control_node')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)  # Publisher for /cmd_vel topic
        self.linear_speed = 0.0  # Initial linear speed
        self.angular_speed = 0.0  # Initial angular speed
        self.msg = Twist()  # Twist message instance

        # Initialize Pygame and joystick module
        pygame.init()
        pygame.joystick.init()

        self.joystick_connected = False  # Status of joystick connection
        self.last_connection_time = "Never"  # Last time the joystick was connected
        self.last_disconnection_time = "Never"  # Last time the joystick was disconnected

        # Diagnostic updater setup
        self.updater = Updater(self)
        self.updater.setHardwareID("joystick")  # Hardware ID for diagnostics
        self.updater.add("Joystick Status", self.joystick_diagnostic)  # Add diagnostic task

        self.stopped = False  # Stop status
        self.max_linear_speed = 0.5  # Maximum linear speed
        self.min_linear_speed = -0.3  # Minimum linear speed
        self.max_angular_speed = 0.5  # Maximum angular speed
        self.min_angular_speed = -0.3  # Minimum angular speed
        self.timer = self.create_timer(0.1, self.update)  # Timer to check joystick input
        self.timer_diag = self.create_timer(1, self.diag_update)  # Timer for diagnostics update

        self.check_joystick_connection()  # Check for joystick connection at startup

    def check_joystick_connection(self):
        # Loop to continuously check joystick connection
        while True:
            pygame.joystick.quit()  # Deinitialize the joystick module
            pygame.joystick.init()  # Reinitialize to rescan devices
            if pygame.joystick.get_count() > 0:
                # Detect and initialize the joystick
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Joystick detected: {self.joystick.get_name()}")
                self.joystick_connected = True
                self.last_connection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                break  # Exit the loop if a joystick is detected
            else:
                if self.joystick_connected:
                    self.last_disconnection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                self.joystick_connected = False
                self.get_logger().warn("No joystick detected. Waiting for connection...")
                time.sleep(2)  # Wait before retrying
            self.updater.update()  # Update diagnostics

    def joy_to_linear_speed(self, joy_val):
        # Convert joystick value to linear speed
        min_joy = -1.0
        max_joy = 1.0
        return joy_to_speed(min_joy, max_joy, self.min_linear_speed, self.max_linear_speed, joy_val)

    def joy_to_angular_speed(self, joy_val):
        # Convert joystick value to angular speed
        min_joy = -1.0
        max_joy = 1.0
        return joy_to_speed(min_joy, max_joy, self.min_angular_speed, self.max_angular_speed, joy_val)

    def apply_deadzone(self, speed):
        # Apply dead zone to joystick input
        if -self.DEADZONE_THRESHOLD < speed < self.DEADZONE_THRESHOLD:
            return 0.0
        return speed

    def update(self):
        # Check if the joystick is still connected
        if pygame.joystick.get_count() == 0:
            if self.joystick_connected:
                self.last_disconnection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.joystick_connected = False
            self.get_logger().warn("Joystick disconnected. Waiting for reconnection...")
            self.check_joystick_connection()

        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:  # Joystick axis movement event
                axis = event.axis
                if axis == self.AXIS_ANGULAR:
                    self.angular_speed = self.joy_to_angular_speed(self.joystick.get_axis(axis))
                    self.angular_speed = self.apply_deadzone(self.angular_speed)
                elif axis == self.AXIS_LINEAR:
                    self.linear_speed = self.joy_to_linear_speed(self.joystick.get_axis(axis))
                    self.linear_speed = self.apply_deadzone(self.linear_speed)
            
            elif event.type == JOYBUTTONDOWN:  # Button press event
                button = event.button
                if button == self.BUTTON_STOP:
                    self.stopped = True
                elif button == self.BUTTON_START:
                    self.stopped = False
                self.get_logger().info(f"Button {button} pressed")

        if self.stopped:  # Stop the robot if the stop button is pressed
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        self.publish_speeds()  # Publish the current speeds

    def diag_update(self):
        # Update diagnostics
        self.updater.update()

    def publish_speeds(self):
        # Publish the current linear and angular speeds to /cmd_vel
        self.msg.linear.x = self.linear_speed
        self.msg.angular.z = self.angular_speed
        self.publisher.publish(self.msg)

    def joystick_diagnostic(self, stat):
        # Diagnostic task for joystick status
        if self.joystick_connected:
            stat.summary(DiagnosticStatus.OK, "Joystick connected")
        else:
            stat.summary(DiagnosticStatus.WARN, "Joystick disconnected")
        
        stat.add("Last connection", self.last_connection_time)
        stat.add("Last disconnection", self.last_disconnection_time)
        return stat

# Function to convert joystick value to speed
def joy_to_speed(min_joy, max_joy, min_speed, max_speed, joy_val):
    return -((max_speed - min_speed) / (max_joy - min_joy)) * joy_val

def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy
    node = JoystickControlNode()  # Create the node
    rclpy.spin(node)  # Spin the node
    rclpy.shutdown()  # Shutdown rclpy

if __name__ == "__main__":
    main()
