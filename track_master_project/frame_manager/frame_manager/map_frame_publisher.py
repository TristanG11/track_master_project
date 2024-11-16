import pymap3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
import numpy as np
class MapFramePublisher(Node):
    def __init__(self):
        super().__init__('map_frame_publisher')
        self.gnss_sub = self.create_subscription(NavSatFix,"/gps/fix",self.gps_cb)
        self.first_fix = NavSatFix()
        self.first_fix_fetched = False
        self.map_frame_computed= False


    def gps_cb(self,msg):
        if not self.first_fix_fetched:
            self.first_fix.header = msg.header
            self.first_fix.latitude = msg.latitude
            self.first_fix.longitude = msg.longitude
            self.first_fix.altitude = msg.latitude
            self.first_fix_fetched = True
        #self.destroy_subscription()


    #once the first fix is fetched let's compute the map frame using enu coordinates : 
    def compute_map_frame(self):
        #first we need to take a random point in a circle of 100m from the robot
        r = np.sqrt(np.random.rand(1)) * 50
        theta = np.random(1)*2*np.pi

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = 0.0

        # then we need to convert this point to enu for the local map we need 
        
        lat, lon, alt = pm.enu2geodetic(x,y,self.first_fix.altitude,self.first_fix.latitude,self.first_fix.longitude,self.first_fix.altitude,pm.Ellipsoid.from_name('WGS84'))


