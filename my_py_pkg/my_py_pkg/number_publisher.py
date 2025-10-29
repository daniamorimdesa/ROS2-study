#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

# Nó para publicar um número, sempre o mesmo no tópico /number

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher") # criar um nó chamado "number_publisher"
        self.publisher_ = self.create_publisher(Int64, "number", 10) # criar um publicador no tópico "number" com fila de tamanho 10
        self.timer_ = self.create_timer(1.0, self.publish_number)    # criar um timer que chama publish_number a cada 1 segundo
        self.get_logger().info("Number Publisher has been started!") # registrar uma mensagem de log

    def publish_number(self):
        msg = Int64() # criar uma mensagem do tipo Int64
        msg.data = 2 # definir o valor do número a ser publicado
        self.publisher_.publish(msg) # publicar a mensagem


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = NumberPublisherNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()