#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station") # criar um nó chamado "robot_news_station"
        self.robot_name = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # criar um publicador no tópico "robot_news" com fila de tamanho 10
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News has been started!")

    def publish_news(self):
        msg = String()
        msg.data = f"Hello, this is {self.robot_name} from the robot news station :)"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = RobotNewsStationNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()