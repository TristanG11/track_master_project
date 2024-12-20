import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
import time
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')

        # Déclaration des paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('twist_topic', '/cmd_vel'),
                ('max_linear_speed', 0.5),
                ('min_linear_speed', -0.3),
                ('max_angular_speed', 0.5),
                ('min_angular_speed', -0.3),
                ('timer_frequency', 0.1),
                ('diag_timer_frequency', 1.0),
                ('min_joy', -1.0),
                ('max_joy', 1.0),
                ('deadzone_threshold', 0.05),
                ('axis_angular', 3),
                ('axis_linear', 4),
                ('button_stop', 0),
                ('button_start', 2),
            ]
        )

        # Récupération des paramètres
        self.twist_topic = self.get_parameter('twist_topic').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.timer_frequency = self.get_parameter('timer_frequency').value
        self.diag_timer_frequency = self.get_parameter('diag_timer_frequency').value
        self.min_joy = self.get_parameter('min_joy').value
        self.max_joy = self.get_parameter('max_joy').value
        self.deadzone_threshold = self.get_parameter('deadzone_threshold').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.button_stop = self.get_parameter('button_stop').value
        self.button_start = self.get_parameter('button_start').value

        # Initialisation du Publisher
        self.publisher = self.create_publisher(Twist, self.twist_topic, 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.msg = Twist()

        # Initialisation de Pygame et du joystick
        pygame.init()
        pygame.joystick.init()

        self.joystick_connected = False
        self.last_connection_time = "Never"
        self.last_disconnection_time = "Never"

        # Initialisation des diagnostics
        self.updater = Updater(self)
        self.updater.setHardwareID("joystick")
        self.updater.add("Joystick Status", self.joystick_diagnostic)

        self.stopped = False

        # Timers
        self.timer = self.create_timer(self.timer_frequency, self.update)
        self.timer_diag = self.create_timer(self.diag_timer_frequency, self.diag_update)

        self.check_joystick_connection()

    def check_joystick_connection(self):
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Joystick detected: {self.joystick.get_name()}")
                self.joystick_connected = True
                self.last_connection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                break
            else:
                if self.joystick_connected:
                    self.last_disconnection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                self.joystick_connected = False
                self.get_logger().warn("No joystick detected. Waiting for connection...")
                time.sleep(2)
            self.updater.update()

    def joy_to_lin(self,joy_val):
        if joy_val >= self.deadzone_threshold :
            coeff = (self.max_linear_speed - self.min_linear_speed) / (self.max_joy - self.deadzone_threshold)
            d = self.min_linear_speed - coeff * self.deadzone_threshold
            return coeff * joy_val + d
        elif joy_val <= -self.deadzone_threshold :
            coeff = (- self.min_linear_speed + self.max_linear_speed) / (-self.deadzone_threshold + self.max_joy )
            d = -self.min_linear_speed  + coeff * self.deadzone_threshold
            return coeff *joy_val + d 
        else:
            return 0.0
    
    def joy_to_ang(self,joy_val):
        if joy_val >= self.deadzone_threshold :
            coeff = (self.max_angular_speed - self.min_angular_speed) / (self.max_joy - self.deadzone_threshold)
            d = self.min_angular_speed - coeff * self.deadzone_threshold
            return coeff * joy_val + d
        elif joy_val <= -self.deadzone_threshold :
            coeff = (- self.min_angular_speed + self.max_angular_speed) / (-self.deadzone_threshold + self.max_joy )
            d = -self.min_angular_speed  + coeff * self.deadzone_threshold
            return coeff *joy_val + d 
        else:
            return 0.0


    def update(self):
        if pygame.joystick.get_count() == 0:
            if self.joystick_connected:
                self.last_disconnection_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.joystick_connected = False
            self.get_logger().warn("Joystick disconnected. Waiting for reconnection...")
            self.check_joystick_connection()

        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                axis = event.axis
                if axis == self.axis_angular:
                    self.angular_speed = - self.joy_to_ang(self.joystick.get_axis(axis))

                elif axis == self.axis_linear:
                    self.linear_speed = - self.joy_to_lin(self.joystick.get_axis(axis))
            elif event.type == JOYBUTTONDOWN:
                button = event.button
                if button == self.button_stop:
                    self.stopped = True
                elif button == self.button_start:
                    self.stopped = False
                self.get_logger().info(f"Button {button} pressed")

        if self.stopped:
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        self.publish_speeds()

    def diag_update(self):
        self.updater.update()

    def publish_speeds(self):
        self.msg.linear.x = self.linear_speed
        self.msg.angular.z = self.angular_speed
        self.publisher.publish(self.msg)

    def joystick_diagnostic(self, stat):
        if self.joystick_connected:
            stat.summary(DiagnosticStatus.OK, "Joystick connected")
        else:
            stat.summary(DiagnosticStatus.WARN, "Joystick disconnected")
        
        stat.add("Last connection", self.last_connection_time)
        stat.add("Last disconnection", self.last_disconnection_time)
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
