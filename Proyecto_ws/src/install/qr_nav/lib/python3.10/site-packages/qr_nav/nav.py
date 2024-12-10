import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class QRMovementController(Node):
    def __init__(self):
        super().__init__('qr_movement_controller')

        # Suscripción al tópico para recibir códigos QR
        self.qr_code_subscription = self.create_subscription(
            String,
            '/qr/code',
            self.qr_code_callback,
            10
        )

        # Publicador al tópico /cmd_vel para controlar el robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variable para almacenar el código QR actual
        self.qr_code = ""

        # Diccionario de acciones basado en el código QR
        self.actions = {
            'A1': 'Der90',
            'B1': 'Izq90',
            'C1': 'Der45',
            'D1': 'Izq45',
            'E1': 'Avanzar',
            'F1': 'Retroceder',
            'G1': 'Alto',
            'H1': 'Vuelta180'
        }

        self.get_logger().info("Nodo QRMovementController iniciado.")

        # Comienza con un giro de 180 grados y luego espera instrucciones
        self.perform_initial_turn()

    def qr_code_callback(self, msg):
        """
        Callback para recibir códigos QR y ejecutar acciones.
        """
        self.qr_code = msg.data
        self.get_logger().info(f"Código QR recibido: {self.qr_code}")
        self.execute_movement()

    def send_twist(self, linear_x, angular_z, duration):
        """
        Publica un mensaje Twist para mover el robot por un tiempo específico.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        self.get_logger().info(
            f"Enviando movimiento: linear_x={linear_x}, angular_z={angular_z}, duration={duration}s"
        )

        # Publicar el mensaje continuamente durante la duración especificada
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)  # Pausa para evitar saturación del tópico

        # Detener el robot al finalizar el tiempo
        self.stop_robot()

    def stop_robot(self):
        """
        Detiene el robot.
        """
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot detenido.")

    def perform_initial_turn(self):
        """
        Realiza un giro inicial de 180 grados.
        """
        self.get_logger().info("Realizando giro inicial de 180 grados.")
        self.send_twist(0.0, 0.5, 1.5)  # Ajustar duración según el robot

    def execute_movement(self):
        """
        Realiza la acción en función del código QR recibido.
        """
        action = self.actions.get(self.qr_code, None)

        if action == 'Der90':
            self.get_logger().info("Acción: Girar 90 grados a la derecha")
            self.send_twist(0.0, 0.5, 1.0)
        elif action == 'Izq90':
            self.get_logger().info("Acción: Girar 90 grados a la izquierda")
            self.send_twist(0.0, -0.5, 1.0)
        elif action == 'Der45':
            self.get_logger().info("Acción: Girar 45 grados a la derecha")
            self.send_twist(0.0, 0.5, 0.5)
        elif action == 'Izq45':
            self.get_logger().info("Acción: Girar 45 grados a la izquierda")
            self.send_twist(0.0, -0.5, 0.5)
        elif action == 'Avanzar':
            self.get_logger().info("Acción: Avanzar en línea recta")
            self.send_twist(0.5, 0.0, 2.0)
        elif action == 'Retroceder':
            self.get_logger().info("Acción: Retroceder en línea recta")
            self.send_twist(-0.5, 0.0, 2.0)
        elif action == 'Alto':
            self.get_logger().info("Acción: Detener el robot")
            self.stop_robot()
        elif action == 'Vuelta180':
            self.get_logger().info("Acción: Girar 180 grados")
            self.send_twist(0.0, 0.5, 2.0)
        else:
            self.get_logger().info("Código QR desconocido. No se realizará acción.")
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    qr_movement_controller = QRMovementController()

    try:
        rclpy.spin(qr_movement_controller)
    finally:
        qr_movement_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

