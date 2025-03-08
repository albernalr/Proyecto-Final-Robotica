#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import *  # Importa la librería de Dynamixel SDK

# 📌 Configuración del puerto y baudrate
DEVICENAME = "/dev/ttyUSB0"  # Ajusta según tu sistema
BAUDRATE = 1000000  # 1 Mbps, recomendado para AX-12A

# 📌 Direcciones de memoria en los motores Dynamixel AX-12A
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34

# 📌 Parámetros del motor
DXL_IDs = [1, 2, 3, 4, 5]  # IDs de los motores del brazo
TORQUE_ENABLE = 1  # Habilitar torque
TORQUE_DISABLE = 0  # Deshabilitar torque
MOVING_SPEED = 25  # Velocidad del motor (0-1023)
TORQUE_LIMITS = {1: 400, 2: 800, 3: 800, 4: 800, 5: 800}  # Límite de torque para cada motor
GOAL_POSITION_DEFAULT = 512  # Posición inicial

class PhantomXController(Node):
    def __init__(self):
        super().__init__('phantomx_controller')

        # 📌 Inicialización del puerto de comunicación
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(1.0)  # AX-12A usa protocolo 1.0

        if not self.portHandler.openPort():
            self.get_logger().error("❌ Error: No se pudo abrir el puerto")
            exit()
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("❌ Error: No se pudo establecer el baudrate")
            exit()

        # 📌 Configurar motores con torque, velocidad y posición inicial
        for dxl_id in DXL_IDs:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_LIMIT, TORQUE_LIMITS[dxl_id])
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MOVING_SPEED, MOVING_SPEED)
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, GOAL_POSITION_DEFAULT)

        self.get_logger().info("✅ Motores configurados con torque, velocidad y posición inicial")

        # 📌 Suscribirse a comandos de posición
        self.subscription = self.create_subscription(
            JointState, '/coppelia/joint_commands', self.joint_command_callback, 10)

        # 📌 Publicador del estado de las articulaciones
        self.publisher = self.create_publisher(JointState, '/phantomx/joint_states', 10)

    def joint_command_callback(self, msg):
        """ Recibe comandos de posición y mueve los motores """
        positions = msg.position  # Recibe un array de posiciones
        if len(positions) < len(DXL_IDs):
            self.get_logger().error("❌ Error: Número de posiciones incorrecto")
            return

        for i, dxl_id in enumerate(DXL_IDs):
            goal_position = int((positions[i] / 3.14) * 512) + 512  # Convertir radianes a valores Dynamixel
            goal_position = max(0, min(1023, goal_position))  # Limitar dentro del rango permitido

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"❌ Error al mover el motor {dxl_id}")
            elif dxl_error:
                self.get_logger().error(f"⚠️ Advertencia en motor {dxl_id}: {dxl_error}")

        self.publish_joint_states()

    def publish_joint_states(self):
        """ Publica la posición actual de los motores """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(DXL_IDs))]
        joint_positions = []

        for dxl_id in DXL_IDs:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error:
                self.get_logger().error(f"❌ No se pudo leer la posición del motor {dxl_id}")
                joint_positions.append(0.0)
            else:
                joint_positions.append((dxl_present_position - 512) * 3.14 / 512)  # Convertir a radianes

        joint_state_msg.position = joint_positions
        self.publisher.publish(joint_state_msg)

    def destroy_node(self):
        """ Apaga los motores y cierra el puerto al finalizar el nodo """
        for dxl_id in DXL_IDs:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = PhantomXController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nApagando el nodo...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

