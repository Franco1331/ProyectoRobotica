import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
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

        # Comienza con un giro de 180 grados y luego avanza hacia el objetivo
        self.perform_movement()

    def qr_code_callback(self, msg):
        self.qr_code = msg.data
        self.get_logger().info(f"Código QR recibido: {self.qr_code}")

    def qr_pos_callback(self, msg):
        self.qr_pos = msg
        self.get_logger().info(f"Posición del QR: x={msg.x}, y={msg.y}, z={msg.z}")

    def qr_size_callback(self, msg):
        self.qr_size = msg.data
        self.get_logger().info(f"Tamaño del QR: {self.qr_size}")

    def send_twist(self, linear_x, angular_z):
        """Publica un mensaje Twist con posibles ajustes en la velocidad de cada motor."""
        twist_msg = Twist()
        # Ajusta las velocidades de los motores si es necesario
        motor_left_speed = linear_x * 0.95  # Reducir ligeramente la velocidad para el motor izquierdo
        motor_right_speed = linear_x * 1.05  # Aumentar ligeramente la velocidad para el motor derecho
        
        twist_msg.linear.x = (motor_left_speed + motor_right_speed) / 2  # Promedio de ambas velocidades
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        """Detiene el robot."""
        self.send_twist(0.0, 0.0)
        self.get_logger().info("Robot detenido.")

    def perform_movement(self):
        """Realiza el giro inicial, avanza hacia el QR y ejecuta la acción correspondiente."""
        self.turn_180_degrees()
        self.advance_to_qr()
        self.execute_movement()

    def turn_180_degrees(self):
        """Gira el robot 180 grados al inicio."""
        self.get_logger().info("Girando 180 grados.")
        self.send_twist(0.0, 0.5)  # Velocidad angular positiva
        time.sleep(2.0)  # Ajustar la duración según el robot
        self.stop_robot()

    def advance_to_qr(self):
        """
        Avanza hacia el QR hasta que el tamaño del QR sea suficientemente grande.
        Ajustado para no depender de un valor tan específico como 714024.
        """
        self.get_logger().info("Avanzando hacia el QR.")

        # Ajuste de velocidad para los motores
        left_velocity = 1.2	  # Ajusta esta velocidad
        right_velocity = 0.5  # Ajusta esta velocidad (si el motor derecho va más rápido)

        # Ajustar la condición para avanzar hacia el QR con un rango más amplio de tamaño
        while self.qr_size <= 300021:  # Condición más flexible para que avance     714024.0
            # Publica los comandos de movimiento (avanza hacia adelante)
            self.send_twist((left_velocity + right_velocity) / 2, 0.0)

            # Pausa breve para evitar saturar el ciclo de control
            time.sleep(0.1)
        
        # Detén el robot cuando se haya alcanzado el tamaño objetivo
        self.stop_robot()
        self.get_logger().info(f"Tamaño objetivo alcanzado ({self.qr_size}). Deteniendo el robot.")

    def execute_movement(self):
        """Realiza la acción en función del código QR recibido."""
        self.get_logger().info("Leyendo acción del código QR.")
        if self.qr_code == 'A1':
            self.get_logger().info("Acción: Girar 90 grados a la derecha")
            self.send_twist(0.0, 0.5)
            time.sleep(1.0)
        elif self.qr_code == 'B1':
            self.get_logger().info("Acción: Girar 90 grados a la izquierda")
            self.send_twist(0.0, -0.5)
            time.sleep(1.0)
        elif self.qr_code == 'C1':
            self.get_logger().info("Acción: Detener el movimiento")
        else:
            self.get_logger().info("Código QR desconocido, no se realizará acción.")
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    qr_movement_controller = QRMovementController()

    rclpy.spin(qr_movement_controller)

    qr_movement_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

