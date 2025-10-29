#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node):

    def __init__(self):
        super().__init__("smartphone") 
        self.subscriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started!")

    def callback_robot_news(self, msg: String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = SmartphoneNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()