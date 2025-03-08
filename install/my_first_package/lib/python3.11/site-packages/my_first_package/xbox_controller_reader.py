#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import numpy as np

def inverse_kinematics(X, Y, Z, phi):
    # Longitudes de los eslabones
    a0 = 0.137
    a1 = 0.105
    a2 = 0.105
    a3 = 0.110
    
    phi1 = np.deg2rad(phi)
    phi1 = -phi1
    
    # Ángulo base
    theta_0 = np.arctan2(Y, X)
    
    # Proyección en el plano XY
    xb = np.sqrt(X**2 + Y**2)
    yb = Z - a0
    
    xa = xb - a3 * np.cos(phi1)
    ya = yb - a3 * np.sin(phi1)
    
    D = np.sqrt(xa**2 + ya**2)
    
    # Verificar alcanzabilidad
    if D > (a1 + a2) or D < abs(a1 - a2):
        raise ValueError("Posición no alcanzable")
    
    cos_theta2 = (D**2 - a1**2 - a2**2) / (2 * a1 * a2)
    theta_2 = np.arccos(cos_theta2)
    theta_2_alt = -theta_2
    
    # Calcular ángulos theta1
    alpha = np.arctan2(ya, xa)
    beta = np.arctan2(a2 * np.sin(theta_2), a1 + a2 * np.cos(theta_2))
    theta_1 = alpha - beta + np.deg2rad(90)
    
    beta_alt = np.arctan2(a2 * np.sin(theta_2_alt), a1 + a2 * np.cos(theta_2_alt))
    theta_1_alt = alpha - beta_alt
    
    # Calcular theta3
    theta_3 = phi1 - theta_1 - theta_2
    theta_3_alt = phi1 - theta_1_alt - theta_2_alt
    
    # Convertir a grados
    theta_0 = np.rad2deg(theta_0)
    theta_1 = np.rad2deg(theta_1_alt) - 90
    theta_2 = np.rad2deg(theta_2_alt)
    theta_3 = np.rad2deg(theta_3_alt)
    
    theta_1 = -theta_1
    theta_2 = -theta_2
    theta_3 = -theta_3
    
    theta_0 = np.radians(theta_0)
    theta_1 = np.radians(theta_1)
    theta_2 = np.radians(theta_2)
    theta_3 = np.radians(theta_3)
    
    return theta_0, theta_1, theta_2, theta_3

class XboxControllerReader(Node):
    def __init__(self):
        super().__init__('xbox_controller_reader')
        self.subscription = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)
        self.publisher = self.create_publisher(JointState, '/coppelia/joint_commands', 10)
        
        self.X = 0
        self.Y = 0.12
        self.Z = 0.12
        self.phi = 45
        self.step = 0.001  # Incremento para X, Y, Z
        self.phi_step = 0.5  # Incremento para phi
        self.gripper_angle = 0.0  # Apertura del efector final
        self.gripper_step = 0.05  # Incremento de apertura/cierre
        
    def listener_callback(self, msg):
        self.X += self.step * msg.axes[0]  # Left Stick X controla X
        self.Y += self.step * msg.axes[1]  # Left Stick Y controla Y
        self.Z += self.step * msg.axes[4]  # Right Stick Y controla Z
        
        if msg.buttons[2]:  # Botón X disminuye phi
            self.phi -= self.phi_step
        if msg.buttons[3]:  # Botón Y aumenta phi
            self.phi += self.phi_step
        
        # Control de apertura del efector final sin limitaciones
        if msg.buttons[0]:  # Botón A abre la garra
            self.gripper_angle += self.gripper_step
        if msg.buttons[1]:  # Botón B cierra la garra
            self.gripper_angle -= self.gripper_step
        
        # 📢 Imprimir posición actual en consola
        self.get_logger().info(f"Posición - X: {self.X:.4f}, Y: {self.Y:.4f}, Z: {self.Z:.4f}, φ: {self.phi:.4f}°, Gripper: {self.gripper_angle:.2f}")
        
        try:
            theta_0, theta_1, theta_2, theta_3 = inverse_kinematics(self.X, self.Y, self.Z, self.phi)
            self.publish_joint_commands([theta_0, theta_1, theta_2, theta_3, self.gripper_angle])
        except ValueError:
            self.get_logger().warn("Posición no alcanzable")
    
    def publish_joint_commands(self, joint_positions):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_flex_joint",
            "arm_wrist_flex_joint",
            "gripper_joint"
        ]
        joint_state_msg.position = joint_positions
        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f"Comandos enviados: {joint_positions}")

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

