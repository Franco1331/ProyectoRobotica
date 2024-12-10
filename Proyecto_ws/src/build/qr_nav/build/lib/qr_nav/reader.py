import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float32
import time


class QRMovementController(Node):
    def __init__(self):
        super().__init__('qr_movement_controller')

        # Suscripciones a los tópicos
        self.qr_code_subscription = self.create_subscription(
            String,
            '/qr/code',
            self.qr_code_callback,
            10
        )

        self.qr_pos_subscription = self.create_subscription(
            Point,
            '/qr/pos',
            self.qr_pos_callback,
            10
        )

        self.qr_size_subscription = self.create_subscription(
            Float32,
            '/qr/size',
            self.qr_size_callback,
            10
        )

        # Publicador al tópico /cmd_vel para controlar el robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables para almacenar el estado actual
        self.qr_code = ""
        self.qr_pos = Point()
        self.qr_size = 0.0

        self.get_logger().info("Nodo QRMovementController iniciado.")

        # Comienza el proceso de movimiento
        self.perform_movement()

    # Callbacks para los tópicos
    def qr_code_callback(self, msg):
        self.qr_code = msg.data
        self.get_logger().info(f"[QR Code] Código recibido: {self.qr_code}")

    def qr_pos_callback(self, msg):
        self.qr_pos = msg
        self.get_logger().info(f"[QR Pos] Posición recibida: x={self.qr_pos.x}, y={self.qr_pos.y}, z={self.qr_pos.z}")

    def qr_size_callback(self, msg):
        self.qr_size = msg.data
        self.get_logger().info(f"[QR Size] Tamaño recibido: {self.qr_size}")

    # Métodos de movimiento
    def send_twist(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        self.send_twist(0.0, 0.0)
        self.get_logger().info("Robot detenido.")

    def perform_movement(self):
        self.get_logger().info("Esperando datos del QR...")
        time.sleep(2)  # Espera 2 segundos para recibir datos
        self.turn_180_degrees()
        self.advance_to_qr()

    def turn_180_degrees(self):
        self.get_logger().info("Girando 180 grados.")
        self.send_twist(0.0, 0.5)  # Velocidad angular positiva
        time.sleep(2.1)
        self.stop_robot()

    def advance_to_qr(self):
        self.get_logger().info("Avanzando hacia el QR.")

        while self.qr_size < 300000:  # Cambia el valor objetivo según lo que esperas
            linear_velocity = 0.5
            angular_velocity = 0.0

            self.send_twist(linear_velocity, angular_velocity)
            self.get_logger().info(f"Avanzando... Tamaño actual del QR: {self.qr_size}")
            time.sleep(0.1)

        self.stop_robot()
        self.get_logger().info("Tamaño objetivo alcanzado. Deteniendo el robot.")


def main(args=None):
    rclpy.init(args=args)
    qr_movement_controller = QRMovementController()

    try:
        rclpy.spin(qr_movement_controller)
    except KeyboardInterrupt:
        qr_movement_controller.get_logger().info("Nodo interrumpido por el usuario.")
    finally:
        qr_movement_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

