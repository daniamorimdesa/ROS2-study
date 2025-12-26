#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

"""
Este node cria uma estação de notícias robótica que publica mensagens de notícias
com o nome do robô e um período de tempo configurável.

- publisher: publica mensagens de notícias em um tópico chamado "robot_news"
"""

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station") # criar um nó chamado "robot_news_station"

        # declarar parâmetros
        self.declare_parameter("robot_name", "C3PO") # declarar o parâmetro "robot_name" com valor padrão "C3PO"
        self.declare_parameter("timer_period", 0.5)  # declarar o parâmetro "timer_period" com valor padrão 0.5 segundos

        # obter os valores dos parâmetros
        self.robot_name = self.get_parameter("robot_name").value     # obter o valor do parâmetro "robot_name"
        self.timer_period = self.get_parameter("timer_period").value # obter o valor do parâmetro "timer_period"

        #self.robot_name = "C3PO" 

        # criar um publisher no tópico "robot_news" com fila de tamanho 10
        self.publisher_ = self.create_publisher(String, "robot_news", 10) 

        # criar um timer que chama publish_news a cada 0.5 segundos
        self.timer_ = self.create_timer(self.timer_period, self.publish_news) 

        self.get_logger().info("Robot News has been started!")  # registrar uma mensagem de log

    # função para publicar notícias
    def publish_news(self): 
        msg = String() # criar uma mensagem do tipo String
        msg.data = f"Hello, this is {self.robot_name} from the robot news station :)"
        self.publisher_.publish(msg) # publicar a mensagem


def main(args=None):
    rclpy.init(args=args)         # inicializar o rclpy
    node = RobotNewsStationNode() # instanciar o nó
    rclpy.spin(node)              # manter o nó ativo
    rclpy.shutdown()              # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()