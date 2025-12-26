#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

"""
Este node cria um nó de "smartphone" que assina mensagens de notícias
publicadas pela estação de notícias robótica.

- subscriber: assina mensagens do tópico "robot_news"
"""
class SmartphoneNode(Node):

    def __init__(self):
        super().__init__("smartphone") 

        # criar um subscriber no tópico "robot_news" com fila de tamanho 10
        self.subscriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)

        self.get_logger().info("Smartphone has been started!")

    # função de callback que processa a mensagem recebida
    def callback_robot_news(self, msg: String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)      # inicializar o rclpy
    node = SmartphoneNode()    # instanciar o nó
    rclpy.spin(node)           # manter o nó ativo 
    rclpy.shutdown()           # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()