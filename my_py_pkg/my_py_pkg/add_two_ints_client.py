#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # importar o serviço AddTwoInts
from functools import partial

"""Este node cria um service client (AddTwoIntsClient) que envia uma request para somar dois inteiros
e processa a response recebida do service (AddTwoInts).

obs: AddTwoInts é um serviço padrão do ROS2 que soma dois inteiros.
"""

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__("add_two_ints_client") # criar um nó chamado "add_two_ints_client"

        # criar um cliente de serviço chamado "add_two_ints" que usa o tipo AddTwoInts
        self.client_ = self.create_client(AddTwoInts, "add_two_ints") 

    # função para chamar o serviço de adicionar dois inteiros
    def call_add_two_ints(self, a, b): 

        # esperar até que o serviço esteja disponível
        while not self.client_.wait_for_service(1.0): 
            self.get_logger().warn("Waiting for Add Two Ints server...") 

        # criar uma requisição do tipo AddTwoInts
        request = AddTwoInts.Request() 
        
        # definir os valores dos inteiros na requisição
        request.a = a 
        request.b = b

        # chamar o serviço de forma assíncrona
        future = self.client_.call_async(request) 
        
        # adicionar uma função de callback para processar a resposta quando estiver pronta
        #future.add_done_callback(self.callback_call_add_two_ints) 
        # se quiser passar argumentos adicionais para o callback, use partial
        future.add_done_callback(partial(self.callback_call_add_two_ints, request=request))

    # função de callback que processa a resposta do serviço
    def callback_call_add_two_ints(self, future, request): 

        # obter o resultado da resposta
        response = future.result()  

        # registrar a soma recebida na resposta
        # self.get_logger().info(f"Got response: {str(response.sum)}") 

        # registrar a soma com os valores da requisição
        self.get_logger().info(f"Result of {request.a} + {request.b} = {response.sum}") 



def main(args=None):
    rclpy.init(args=args)          # inicializar o rclpy
    node = AddTwoIntsClient()      # instanciar o nó
    node.call_add_two_ints(3, 7)   # chamar o serviço para adicionar 3 e 7
    node.call_add_two_ints(1, 4)   # chamar o serviço para adicionar 1 e 4
    node.call_add_two_ints(50, 50) # chamar o serviço para adicionar 50 e 50
    rclpy.spin(node)               # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()               # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()