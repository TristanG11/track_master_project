#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_utils.msg import WheelCommands
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus  # Import nécessaire

class WheelCommandsDebug(Node):
    def __init__(self):
        super().__init__("wheel_commands_debug")
        self.wheel_separation = None
        self.wheel_radius = None

        pkg_path = get_package_share_directory('robot_control')
        yaml_file_path = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

        with open(yaml_file_path, 'r') as yaml_file:
            try:
                config = yaml.safe_load(yaml_file)
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la lecture du fichier YAML : {e}")
                return
        

        diff_drive_params = config.get('diff_drive_controller', {}).get('ros__parameters', {})
        self.wheel_separation = diff_drive_params.get('wheel_separation')
        self.wheel_radius = diff_drive_params.get('wheel_radius')

        self.wheel_cmd_sub = self.create_subscription(WheelCommands, "cmd_vel_published", self.wheel_cmd_cb, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/diff_drive_controller/cmd_vel_unstamped", self.cmd_vel_cb, 10)

        self.wheel_cmd = WheelCommands()
        self.cmd_vel = Twist()
        self.timer = self.create_timer(0.2, self.update)  # 5 Hz

        self.updater = Updater(self)
        self.updater.setHardwareID("wheel_commands_debug")
        self.updater.add("Wheel Command Diagnostics", self.diagnostic_task)

    def wheel_cmd_cb(self, msg):
        self.wheel_cmd = msg

    def cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def compute_cmd_vel(self):
        vx = self.cmd_vel.linear.x
        wz = self.cmd_vel.angular.z

        vfl = (vx - wz * self.wheel_separation / 2.0) / self.wheel_radius
        vfr = (vx + wz * self.wheel_separation / 2.0) / self.wheel_radius
        vrl = (vx - wz * self.wheel_separation / 2.0) / self.wheel_radius
        vrr = (vx + wz * self.wheel_separation / 2.0) / self.wheel_radius

        return (vfl, vfr, vrl, vrr)

    def update(self):

        self.updater.update()

    def diagnostic_task(self, stat):
        computed_velocities = self.compute_cmd_vel()

        published_velocities = [
            self.wheel_cmd.front_left_wheel_speed.data,
            self.wheel_cmd.front_right_wheel_speed.data,
            self.wheel_cmd.rear_left_wheel_speed.data,
            self.wheel_cmd.rear_right_wheel_speed.data,
        ]

        stat.summary(DiagnosticStatus.OK, "Comparaison des vitesses des roues")

        wheel_names = ["Avant Gauche", "Avant Droite", "Arrière Gauche", "Arrière Droite"]
        for i in range(4):
            stat.add(f"Vitesse Calculée ({wheel_names[i]})", f"{computed_velocities[i]:.3f}")
            stat.add(f"Vitesse Publiée ({wheel_names[i]})", f"{published_velocities[i]:.3f}")
            difference = computed_velocities[i] - published_velocities[i]
            stat.add(f"Différence ({wheel_names[i]})", f"{difference:.3f}")
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandsDebug()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
