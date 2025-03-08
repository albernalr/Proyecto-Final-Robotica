#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

class XboxControllerReader(Node):
    def __init__(self):
        super().__init__('xbox_controller_reader')

        # Suscripción a los mensajes de Joy para recibir la entrada del controlador
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        
        # Publicador para el tema /coppelia/joint_commands
        self.publisher = self.create_publisher(JointState, '/coppelia/joint_commands', 10)

        # Nombres de los botones y ejes
        self.button_names = [
            'A', 'B', 'X', 'Y',
            'LB', 'RB',
            'Back', 'Start',
            'Guide',
            'Left Stick', 'Right Stick'
        ]
        
        self.axis_names = [
            'Left Stick X',
            'Left Stick Y',
            'LT',
            'Right Stick X',
            'Right Stick Y',
            'RT',
            'D-Pad X',
            'D-Pad Y'
        ]
        
        # Inicialización de las posiciones de las articulaciones
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # Inicializando a 0 radianes
        self.joint_names = [
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_flex_joint",
            "arm_wrist_flex_joint"
        ]
    
    def listener_callback(self, msg):
        # Comprobamos los botones presionados para sumar o restar grados
        if msg.buttons[0]:  # Botón A para aumentar posiciones
            self.change_joint_positions(increase=True)
        elif msg.buttons[1]:  # Botón B para disminuir posiciones
            self.change_joint_positions(increase=False)

        # Publicamos los nuevos valores de las articulaciones
        self.publish_joint_commands()

    def change_joint_positions(self, increase=True):
        """Función para cambiar las posiciones de las articulaciones."""
        step = 0.1  # Incremento o decremento en radianes
        
        if increase:
            # Aumentar las posiciones de las articulaciones
            self.joint_positions = [pos + step for pos in self.joint_positions]
        else:
            # Disminuir las posiciones de las articulaciones
            self.joint_positions = [pos - step for pos in self.joint_positions]
        
        self.get_logger().info(f"Posiciones de las articulaciones: {self.joint_positions}")

    def publish_joint_commands(self):
        """Publicar los comandos de las articulaciones en el tema '/coppelia/joint_commands'"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        
        # Publicamos el mensaje
        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f"Comandos de articulaciones enviados: {self.joint_positions}")

    def clear_screen(self):
        """Limpiar la pantalla de la consola"""
        print("\033[2J\033[H", end="")

def main(args=None):
    rclpy.init(args=args)
    controller_reader = XboxControllerReader()
    
    try:
        rclpy.spin(controller_reader)
    except KeyboardInterrupt:
        print("\nApagando el nodo...")
    finally:
        controller_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

