#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0 # inicializar o contador
        self.publisher_ = self.create_publisher(Int64, "number_count", 10) # criar um publicador no tópico "number_count" com fila de tamanho 10
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10) # criar um assinante no tópico "number" com fila de tamanho 10
        self.get_logger().info("Number Counter has been started!") # registrar uma mensagem de log

    def callback_number(self, msg: Int64):

        # somar o valor recebido ao contador
        self.counter_ += msg.data
        
        #self.get_logger().info(f"Counter: {self.counter_}")

        # publicar o valor do contador sempre que ele for atualizado
        output = Int64() # criar uma mensagem do tipo Int64
        output.data = self.counter_ # definir valor do contador como dado a ser publicado
        self.publisher_.publish(output) # publicar a mensagem
        self.get_logger().info(f"Received: {msg.data} | Published counter: {self.counter_}") # registrar o valor recebido|publicado

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = NumberCounterNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
    
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()