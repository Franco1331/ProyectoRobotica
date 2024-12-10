import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publica a 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Cambia 0 si tu cámara tiene otro índice
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convertir el frame de OpenCV a un mensaje Image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando imagen en /camera')
        else:
            self.get_logger().error('No se pudo capturar imagen de la cámara')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
