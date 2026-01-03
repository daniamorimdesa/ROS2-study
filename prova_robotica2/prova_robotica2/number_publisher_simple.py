#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

"""
Este node é um publisher que publica um número inteiro em um tópico
chamado /number a cada intervalo de tempo definido.

- publisher: publica no tópico "number"
"""

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher") # criar um nó chamado "number_publisher"

        # criar um publicador no tópico "number" com fila de tamanho 1
        self.publisher_ = self.create_publisher(Int64, "number", 1)  

        self.timer_period_ = 1.0  # segundos (definir o período do timer para 1 segundo)
        self.number_ = 1          # número a ser publicado

        # criar um timer que chama publish_number a cada timer_period_ segundos
        self.timer_ = self.create_timer(self.timer_period_, self.publish_number) 

        self.get_logger().info("Number Publisher has been started!")  

    # função para publicar o número
    def publish_number(self): 
        msg = Int64()                 # criar uma mensagem do tipo Int64
        msg.data = self.number_       # definir o valor do número a ser publicado
        self.publisher_.publish(msg)  # publicar a mensagem

def main(args=None):
    rclpy.init(args=args)         # inicializar o rclpy
    node = NumberPublisherNode()  # instanciar o nó
    rclpy.spin(node)              # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()              # finalizar o rclpy
#----------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()