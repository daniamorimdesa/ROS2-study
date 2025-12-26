#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # importar o serviço AddTwoInts

"""
Este node cria um service server (AddTwoIntsServer) que processa requisições para 
somar dois inteiros e retorna a resposta com a soma calculada.

obs: AddTwoInts é um serviço padrão do ROS2 que soma dois inteiros.
"""

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__("add_two_ints_server") # criar um nó chamado "add_two_ints_server"

        # criar um servidor de serviço chamado "add_two_ints" que usa o tipo AddTwoInts 
        # e chama callback_add_two_ints quando uma requisição é recebida
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints) 

        self.get_logger().info("Add Two Ints server has been started!") # registrar uma mensagem de log

    # função de callback que processa a requisição e retorna a resposta
    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response): 

        # calcular a soma dos dois inteiros recebidos na requisição
        response.sum = request.a + request.b 

        self.get_logger().info(str(request.a)+ " + " + str(request.b) + " = " + str(response.sum))

        return response # retornar a resposta com a soma calculada


def main(args=None):
    rclpy.init(args=args)     # inicializar o rclpy
    node = AddTwoIntsServer() # instanciar o nó
    rclpy.spin(node)          # manter o nó ativo para processar callbacks
    rclpy.shutdown()          # finalizar o rclpy
#-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()