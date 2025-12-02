#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot

class MoveRobotClientNode(Node):

    def __init__(self):
        super().__init__("move_robot_client") # criar um nó chamado "move_robot_client"


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MoveRobotClientNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()