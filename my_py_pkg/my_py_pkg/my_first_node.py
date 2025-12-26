#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

"""Este node cria um nó simples que registra "Hello world" ao iniciar 
e depois registra "Hello" seguido de um contador a cada segundo.
"""
class MyNode(Node):

    def __init__(self):
        super().__init__("py_test") # criar um nó chamado "py_test"

        self.counter_ = 0

        self.get_logger().info("Hello world") # registrar uma mensagem de log
        
        # criar um timer que chama timer_callback a cada 1 segundo
        self.create_timer(1.0, self.timer_callback) 

    # função chamada pelo timer a cada segundo
    def timer_callback(self): 
        self.get_logger().info("Hello" + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MyNode()       # instanciar o nó
    rclpy.spin(node)      # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()      # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()