#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool # importar o serviço SetBool

"""Este node cria um contador que recebe números inteiros de um tópico, 
acumula esses números e publica o total em outro tópico.
Também oferece um serviço para resetar o contador.

- publisher: publica o valor do contador no tópico "number_count"
- subscriber: assina números inteiros do tópico "number"
- service server: oferece um serviço chamado "reset_counter" para resetar o contador
""" 
class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter") # criar um nó chamado "number_counter"

        self.counter_ = 0 # inicializar o contador

        # criar um publisher no tópico "number_count" com fila de tamanho 10
        self.publisher_ = self.create_publisher(Int64, "number_count", 10) 

        # criar um subscriber no tópico "number" com fila de tamanho 10
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10) 

        # criar um servidor de serviço chamado "reset_counter" que usa o tipo SetBool 
        # e chama callback_reset_counter quando uma requisição é recebida
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter) 

        self.get_logger().info("Number Counter has been started!") 

    # função de callback que processa a mensagem recebida
    def callback_number(self, msg: Int64): 

        # somar o valor recebido ao contador
        self.counter_ += msg.data
        
        #self.get_logger().info(f"Counter: {self.counter_}")

        # publicar o valor do contador sempre que ele for atualizado
        output = Int64()                # criar uma mensagem do tipo Int64
        output.data = self.counter_     # definir valor do contador como dado a ser publicado
        self.publisher_.publish(output) # publicar a mensagem

        # registrar o valor recebido|publicado
        self.get_logger().info(f"Received: {msg.data} | Published counter: {self.counter_}") 
    
    # função de callback que processa a requisição do serviço de resetar o contador
    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):

        # se o dado na requisição for True, resetar o contador
        if request.data: 
            self.counter_ = 0                   # resetar o contador para zero
            response.success = True             # definir sucesso como True na resposta
            response.message = "Counter reset!" # definir mensagem na resposta

            self.get_logger().info("Counter has been reset to zero!") 

        else:
            response.success = False                          # definir sucesso como False na resposta
            response.message = "Counter reset not performed." # definir mensagem na resposta

            self.get_logger().info("Counter has not been reset...") 

        return response # retornar a resposta


def main(args=None):
    rclpy.init(args=args)      # inicializar o rclpy
    node = NumberCounterNode() # instanciar o nó
    rclpy.spin(node)           # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()           # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()