#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from track_master_debug.srv import GetCmdType


class AppInitializer(Node):
    def __init__(self):
        super().__init__("app_initializer")

        # Variable locale pour stocker la dernière commande
        self.current_cmd_type = ""

        # Subscriber à /cmd_type
        self.cmd_type_sub = self.create_subscription(
            String,
            "/cmd_type",
            self.cmd_type_callback,
            10
        )

        # Service /get_cmd_type
        self.cmd_type_srv = self.create_service(
            GetCmdType,
            "/app_initializer/get_cmd_type",
            self.get_cmd_type_callback
        )

        self.get_logger().info("AppInitializer node started. Waiting for service calls...")

    def cmd_type_callback(self, msg: String):
        """Met à jour la valeur courante de cmd_type"""
        self.current_cmd_type = msg.data
        self.get_logger().info(f" /cmd_type updated: {self.current_cmd_type}")

    def get_cmd_type_callback(self, request, response):
        self.get_logger().info(f" Service /get_cmd_type called, returning: {self.current_cmd_type}")
        response.cmd_type = self.current_cmd_type
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AppInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
