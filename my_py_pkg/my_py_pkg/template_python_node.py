#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):

    def __init__(self):
        super().__init__("node_name") # criar um nó chamado "node_name"

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MyCustomNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()