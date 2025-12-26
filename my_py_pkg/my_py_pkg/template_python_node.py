#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

"""
Este é um template básico para criar um nó em Python usando ROS2.
Basta preencher o nome do nó e adicionar funcionalidades conforme necessário.
"""
class MyCustomNode(Node):

    def __init__(self):
        super().__init__("node_name") # criar um nó chamado "node_name"

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MyCustomNode() # instanciar o nó
    rclpy.spin(node)      # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()      # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()