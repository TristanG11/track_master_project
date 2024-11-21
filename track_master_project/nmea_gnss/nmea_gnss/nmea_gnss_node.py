#!/usr/bin/env python3

from nmea_gnss.nmea_parser import * 
import serial
import rclpy 
import rclpy.time
from msg_utils.msg import GpsVelocityHeading
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import time
import math
class NmeaGnssNode(Node):
    def __init__(self):
        super().__init__('nmea_gnss_node')
        self.nav_sat_fix = NavSatFix() 
        self.heading_vel = GpsVelocityHeading()
        self.gnss_stream = None

        self.declare_parameter('/gnss/module_port', '/dev/ttyUSB0')
        self.declare_parameter('/gnss/baudrate', 38400)
        self.gnss_module_port = self.get_parameter_or('/gnss/module_port', '/dev/ttyUSB0').value
        self.gnss_module_baudrate = self.get_parameter_or('/gnss/baudrate', 38400).value


        self.gnss_serial = serial.Serial(self.gnss_module_port,self.gnss_module_baudrate, timeout=1)
        self.nav_sat_fix_publisher = self.create_publisher(NavSatFix,'/gnss/fix',10)
        self.gnss_vel_heading_publisher = self.create_publisher(GpsVelocityHeading,'/gnss/heading_vel',10)

        self.create_timer(0.1,self.get_gnss_stream)
        
    
    def get_gnss_stream(self):
            try : 
                raw_data = self.gnss_serial.readline().decode('utf-8')
                if raw_data:
                    self.gnss_stream = raw_data
                    self.publish_data()
            except Exception as e:
                self.get_logger().error(str(e))


    def publish_data(self):
        if self.gnss_stream: 
            try:
                if 'GGA' in self.gnss_stream:
                    parsed_data = parse_gga(self.gnss_stream)
                    self.nav_sat_fix.header.stamp = self.get_clock().now().to_msg()
                    self.nav_sat_fix.status.status = map_gga_to_status(parsed_data['fix_quality'])
                    self.nav_sat_fix.status.service = parsed_data['service'] 
                    self.nav_sat_fix.latitude = parsed_data['latitude']
                    self.nav_sat_fix.longitude = parsed_data['longitude']
                    self.nav_sat_fix.altitude = parsed_data['altitude']
                    self.nav_sat_fix_publisher.publish(self.nav_sat_fix)
                if 'VTG' in self.gnss_stream:
                    parsed_data = parse_vtg(self.gnss_stream)
                    if any(value is None for value in parsed_data.values()):
                        self.heading_vel.valid.data = False
                    else:
                        self.heading_vel.valid.data = True
                        self.heading_vel.heading.data = parsed_data['true_course'] * math.pi / 180.0
                        self.heading_vel.velocity.data = parsed_data['speed_ms']
                    self.gnss_vel_heading_publisher.publish(self.heading_vel)
            except Exception as e:
                self.get_logger().error(str(e))

def main():
    rclpy.init(args=None)
    node = NmeaGnssNode()
    try :
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info("Shutting down node...")
        node.get_logger().error(str(e))
    finally:
        node.gnss_serial.close()
        rclpy.shutdown()
        

    
if __name__ == "__main__":
    main()


