import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from yolo_msgs.msg import DetectionArray, KeyPoint3D, KeyPoint3DArray
import math
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
        self.target_pub = self.create_publisher(Pose2D, "/track_target",qos_profile)
        # Publication
        self.pub_detections_3d = self.create_publisher(DetectionArray, '/detections_3d', qos_profile)

        self.bridge = CvBridge()
        self.depth_img = None
        self.depth_info = None

        # Dimensions image couleur et profondeur
        self.sx = 640/1280.0
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

        fx = self.depth_info.k[0]
        fy = self.depth_info.k[4]
        cx = self.depth_info.k[2]
        cy = self.depth_info.k[5]

        detections_out = DetectionArray()
        detections_out.header = msg.header
        #detections_out.frame_id = msg.frame_id

        for det in msg.detections:
            det_out = det  # copie du detection original
            keypoints3d = KeyPoint3DArray()
            keypoints3d.frame_id = self.depth_info.header.frame_id

            for kp in det.keypoints.data:
                print(kp.id)

                if kp.id !=16:
                    continue

                
                x_color = kp.point.x
                y_color = kp.point.y

                # Conversion vers coords de l'image profondeur
                u = int(round(x_color * self.sx))
                v = int(round(y_color * self.sy))
                print("u", u)
                print("v", v)
                #u = np.clip(u, 0, self.w_depth - 1)
                #v = np.clip(v, 0, self.h_depth - 1)

                max_value = np.max(self.depth_img)
                #print("Valeur maximale :", max_value)
                
                Z = self.depth_img[v, u] / 1000.0  # mm -> m

                if Z<= 0:
                    continue
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                target_msg = Pose2D()
                target_msg.x = Z
                target_msg.y = X
                target_msg.theta = math.atan2(target_msg.y,target_msg.x)
                
                theta_deg = math.degrees(target_msg.theta)
                print(theta_deg)
                #self.target_pub.publish(target_msg)

                print('(x,y,z) = ',X,Y,Z)

                kp3d = KeyPoint3D()
                kp3d.id = kp.id
                kp3d.point.x = float(X)
                kp3d.point.y = float(Y)
                kp3d.point.z = float(Z)
                kp3d.score = kp.score
                keypoints3d.data.append(kp3d)

            det_out.keypoints3d = keypoints3d
            detections_out.detections.append(det_out)

            self.pub_detections_3d.publish(detections_out)
            self.get_logger().info(f"Published {len(detections_out.detections)} detections_3d")

def main(args=None):
    rclpy.init(args=args)
    node = Detections3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
