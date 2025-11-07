#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("turtle_spawner") # criar um nó chamado "turtle_spawner"

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleSpawnerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()