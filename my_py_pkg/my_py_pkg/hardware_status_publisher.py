#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

# Nó para publicar usando a interface de mensagem criada HardwareStatus

class HardwareStatusPublisherNode(Node):

    def __init__(self):
        super().__init__("hardware_status_publisher") # criar um nó chamado "hardware_status_publisher"
        self.hw_status_publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10) # criar um publicador no tópico "hardware_status" com fila de tamanho 10
        self.timer_ = self.create_timer(1.0, self.publish_hardware_status)    # criar um timer que chama publish_hardware_status a cada 1 segundo
        self.get_logger().info("Hardware Status Publisher has been started!") # registrar uma mensagem de log

    def publish_hardware_status(self):
        msg = HardwareStatus() # criar uma mensagem do tipo HardwareStatus
        msg.temperature = 43.7  # definir a temperatura
        msg.are_motors_ready = True  # definir o status dos motores
        msg.debug_message = "Nothing special"  # definir a mensagem de depuração
        self.hw_status_publisher_.publish(msg) # publicar a mensagem


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = HardwareStatusPublisherNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()