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

        # Configuración PID
        self.kp = 0.01  # Constante proporcional
        self.ki = 0.0   # Constante integral
        self.kd = 0.001 # Constante derivativa
        self.integral = 0.0
        self.prev_error = 0.0
        self.target_x = 290  # Centro objetivo en píxeles
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info("Nodo QRMovementController iniciado.")

        # Comienza con un giro de 180 grados y luego avanza hacia el objetivo
        self.perform_movement()

    def qr_code_callback(self, msg):
        self.qr_code = msg.data
        self.get_logger().info(f"Código QR recibido: {self.qr_code}")

    def qr_pos_callback(self, msg):
        self.qr_pos = msg

    def qr_size_callback(self, msg):
        self.qr_size = msg.data

    def send_twist(self, linear_x, angular_z):
        """Publica un mensaje Twist."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        """Detiene el robot."""
        self.send_twist(0.0, 0.0)
        self.get_logger().info("Robot detenido.")

    def pid_control(self, error):
        """Controlador PID para ajustar la posición del robot basado en la posición del QR."""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        delta_time = current_time - self.last_time
        delta_error = error - self.prev_error

        # Calcular términos PID
        self.integral += error * delta_time
        derivative = delta_error / delta_time if delta_time > 0 else 0.0

        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Actualizar valores previos
        self.prev_error = error
        self.last_time = current_time

        return control

    def perform_movement(self):
        """Realiza el giro inicial, avanza hacia el QR y ejecuta la acción correspondiente."""
        self.turn_180_degrees()
        self.advance_to_qr()
        self.execute_movement()

    def turn_180_degrees(self):
        """Gira el robot 180 grados al inicio."""
        self.get_logger().info("Girando 180 grados.")
        self.send_twist(0.0, 0.5)  # Velocidad angular positiva
        time.sleep(1.6)  # Ajustar la duración según el robot
        self.stop_robot()

    def advance_to_qr(self):
        """
        Avanza hacia el QR usando PID para mantener el robot recto,
        hasta que el tamaño del QR sea mayor o igual a 64400.
        """
        self.get_logger().info("Avanzando hacia el QR.")
        while self.qr_size <= 64400:
            error = self.target_x - self.qr_pos.x
            control_signal = self.pid_control(error)

            # Avanzar y corregir el ángulo del robot
            self.send_twist(0.2, control_signal)  # Ajustar velocidad lineal y control angular
            time.sleep(0.1)  # Pausa breve para evitar sobrecarga

        self.stop_robot()
        self.get_logger().info("Tamaño objetivo alcanzado. Deteniendo el robot.")

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
