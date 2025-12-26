#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64

"""
Este node é um publisher que publica um número inteiro em um tópico
chamado /number a cada intervalo de tempo definido.
O número e o intervalo de tempo podem ser configurados via parâmetros.

- publisher: publica no tópico "number"
"""

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher")                          # criar um nó chamado "number_publisher"

        # declarar parâmetros
        self.declare_parameter("number", 2)                           # declarar o parâmetro "number" com valor padrão 2
        self.declare_parameter("timer_period", 1.0)                   # declarar o parâmetro "timer_period" com valor padrão 1.0 segundos

        # obter os valores dos parâmetros
        self.number_ = self.get_parameter("number").value             # obter o valor do parâmetro "number"
        self.timer_period_ = self.get_parameter("timer_period").value # obter o valor do parâmetro "timer_period"

        # adicionar callback para mudanças de parâmetros
        self.add_on_set_parameters_callback(self.parameters_callback)

        # criar um publicador no tópico "number" com fila de tamanho 10
        self.publisher_ = self.create_publisher(Int64, "number", 10)  

        # criar um timer que chama publish_number a cada timer_period_ segundos
        self.timer_ = self.create_timer(self.timer_period_, self.publish_number) 

        self.get_logger().info("Number Publisher has been started!")  

    # função para publicar o número
    def publish_number(self): 
        msg = Int64()                 # criar uma mensagem do tipo Int64
        msg.data = self.number_       # definir o valor do número a ser publicado
        self.publisher_.publish(msg)  # publicar a mensagem

    # callback para mudanças de parâmetros
    def parameters_callback(self, params: list[Parameter]): 
        for param in params:
            if param.name == "number":
                self.number_ = param.value
                self.get_logger().info(f"Parameter 'number' changed to: {self.number_}")


def main(args=None):
    rclpy.init(args=args)         # inicializar o rclpy
    node = NumberPublisherNode()  # instanciar o nó
    rclpy.spin(node)              # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()              # finalizar o rclpy
#----------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()