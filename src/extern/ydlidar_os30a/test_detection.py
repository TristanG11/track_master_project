import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped 
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from yolo_msgs.msg import DetectionArray
import math
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_from_euler

class Detections3DNode(Node):
    def __init__(self):
        super().__init__('detections_3d_node')

        # Souscriptions
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Image, '/apc/depth/image_raw', self.depth_callback, qos_profile)
        self.create_subscription(CameraInfo, '/apc/depth/camera_info', self.info_callback, qos_profile)
        self.create_subscription(DetectionArray, '/yolo/detections', self.detections_callback, qos_profile)
        # Publication
        self.pub = self.create_publisher(PoseStamped, 'person_goal', qos_profile)

        self.bridge = CvBridge()
        self.depth_img = None
        self.depth_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        # Dimensions image couleur et profondeur
        self.sx = 640 / 1280.0
        self.sy = 460 / 920.0

        #self.h_color, self.w_color = 920, 1280
        #self.h_depth, self.w_depth = 460, 640

        self.get_logger().info("detections_3d_node started")

    def depth_callback(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def info_callback(self, msg: CameraInfo):
        self.depth_info = msg

    def detections_callback(self, msg: DetectionArray):

        if self.depth_img is None or self.depth_info is None:
            return  # pas encore prêt

        fx = self.depth_info.k[0] * 0.5
        fy = self.depth_info.k[4] * 0.5
        cx = self.depth_info.k[2] * 0.5
        cy = self.depth_info.k[5] * 0.5 

        #detections_out = DetectionArray()
        #detections_out.header = msg.header
        #detections_out.frame_id = msg.frame_id

        for det in msg.detections:
            for kp in det.keypoints.data:

                if kp.id !=16:
                    continue

                
                x_color = kp.point.x
                y_color = kp.point.y

                # Conversion vers coords de l'image profondeur
                u = int(round(x_color * self.sx))
                v = int(round(y_color * self.sy))
                print("u", u)
                print("v", v)
                
                Z = self.depth_img[v, u] / 1000.0  # mm -> m

                if Z<= 0:
                    continue

                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                print('(x,y,z) = ',X,Y,Z)

                p_cam = PointStamped()
                p_cam.header.frame_id = 'depth_frame'
                p_cam.header.stamp = self.get_clock().now().to_msg()
                p_cam.point.x = Z
                p_cam.point.y = -X
                p_cam.point.z = 0 # pas important
                
                try : 
                    tf = self.tf_buffer.lookup_transform('odom','depth_frame',rclpy.time.Time())
                    p_in_odom = do_transform_point(p_cam,tf)

                    dx = p_in_odom.point.x
                    dy = p_in_odom.point.y
                    yaw = math.atan2(dy,dx)

                    q = quaternion_from_euler(0,0,yaw)

                    # Fill pose
                    pose = PoseStamped()
                    pose.header.frame_id = 'odom'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = p_in_odom.point.x
                    pose.pose.position.y = p_in_odom.point.y
                    pose.pose.position.z = p_in_odom.point.z
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    self.pub.publish(pose)

                except Exception as e:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                
def main(args=None):
    rclpy.init(args=args)
    node = Detections3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
