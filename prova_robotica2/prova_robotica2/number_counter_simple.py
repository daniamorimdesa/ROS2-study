#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

"""Este node cria um contador que recebe números inteiros de um tópico, 
acumula esses números e publica o total em outro tópico.
- publisher: publica o valor do contador no tópico "number_count"
- subscriber: assina números inteiros do tópico "number"
""" 
class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter") # criar um nó chamado "number_counter"

        self.counter_ = 0 # inicializar o contador

        # criar um publisher no tópico "number_count" com fila de tamanho 1
        self.publisher_ = self.create_publisher(Int64, "number_count", 1) 

        # criar um subscriber no tópico "number" com fila de tamanho 1
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 1) 

        self.get_logger().info("Number Counter has been started!") 

    # função de callback que processa a mensagem recebida
    def callback_number(self, msg: Int64): 

        # somar o valor recebido ao contador
        self.counter_ += msg.data

        # publicar o valor do contador sempre que ele for atualizado
        output = Int64()                # criar uma mensagem do tipo Int64
        output.data = self.counter_     # definir valor do contador como dado a ser publicado
        self.publisher_.publish(output) # publicar a mensagem

        # registrar o valor recebido|publicado
        self.get_logger().info(f"Received: {msg.data} | Published counter: {self.counter_}") 
    
def main(args=None):
    rclpy.init(args=args)      # inicializar o rclpy
    node = NumberCounterNode() # instanciar o nó
    rclpy.spin(node)           # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()           # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()