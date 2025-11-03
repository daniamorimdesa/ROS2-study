#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station") # criar um nó chamado "robot_news_station"

        # declarar parâmetros
        self.declare_parameter("robot_name", "C3PO") # declarar o parâmetro "robot_name" com valor padrão "C3PO"
        self.robot_name = self.get_parameter("robot_name").value # obter o valor do parâmetro "robot_name"
        #self.robot_name = "C3PO" 

        self.publisher_ = self.create_publisher(String, "robot_news", 10) # criar um publicador no tópico "robot_news" com fila de tamanho 10
        self.timer_ = self.create_timer(0.5, self.publish_news) # criar um timer que chama publish_news a cada 0.5 segundos
        self.get_logger().info("Robot News has been started!")  # registrar uma mensagem de log

    def publish_news(self):
        msg = String() # criar uma mensagem do tipo String
        msg.data = f"Hello, this is {self.robot_name} from the robot news station :)"
        self.publisher_.publish(msg) # publicar a mensagem


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = RobotNewsStationNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()