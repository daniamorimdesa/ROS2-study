#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__("add_two_ints_client") # criar um nó chamado "add_two_ints_client"
        self.client_ = self.create_client(AddTwoInts, "add_two_ints") # criar um cliente de serviço chamado "add_two_ints" que usa o tipo AddTwoInts

    def call_add_two_ints(self, a, b): # função para chamar o serviço de adicionar dois inteiros
        while not self.client_.wait_for_service(1.0): # esperar até que o serviço esteja disponível
            self.get_logger().warn("Waiting for Add Two Ints server...") # registrar uma mensagem de log enquanto espera
        request = AddTwoInts.Request() # criar uma requisição do tipo AddTwoInts
        # definir os valores dos inteiros na requisição
        request.a = a 
        request.b = b

        future = self.client_.call_async(request) # chamar o serviço de forma assíncrona
        
        # adicionar uma função de callback para processar a resposta quando estiver pronta
        #future.add_done_callback(self.callback_call_add_two_ints) 
        # se quiser passar argumentos adicionais para o callback, use partial
        future.add_done_callback(partial(self.callback_call_add_two_ints, request=request))
    
    def callback_call_add_two_ints(self, future, request): # função de callback que processa a resposta do serviço
        response = future.result()                # obter o resultado da resposta
        # self.get_logger().info(f"Got response: {str(response.sum)}") # registrar a soma recebida na resposta
        self.get_logger().info(f"Result of {request.a} + {request.b} = {response.sum}") # registrar a soma com os valores da requisição



def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = AddTwoIntsClient() # instanciar o nó
    node.call_add_two_ints(3, 7) # chamar o serviço para adicionar 3 e 7
    node.call_add_two_ints(1, 4) # chamar o serviço para adicionar 1 e 4
    node.call_add_two_ints(50, 50) # chamar o serviço para adicionar 50 e 50
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()