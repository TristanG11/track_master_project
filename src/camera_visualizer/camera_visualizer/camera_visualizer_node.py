import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_visualizer_node')
        # Publisher sur CompressedImage
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            1
        )

        # Timer pour la fréquence de publication (10 Hz)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Capture de la caméra
        self.cap = cv2.VideoCapture("/dev/video0")

        # Limiter la résolution pour réduire le flux
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Impossible de lire la caméra")
            return

        # Encoder en JPEG avec qualité 70%
        msg = CompressedImage()
        msg.format = "jpeg"
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        msg.data = buffer.tobytes()
         
        #FPS = 20;
        #flux_s = len(msg.data) * FPS  # bytes/s
        #flux_MB_s = flux_s / 1024 / 1024
        #print(flux_MB_s) 
        self.publisher.publish(msg)
        #self.get_logger().debug(f"Published frame size: {len(msg.data)} bytes")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
