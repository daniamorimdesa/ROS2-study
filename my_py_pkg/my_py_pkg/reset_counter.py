#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool # importar o serviço SetBool
from functools import partial

class ResetCounterClient(Node):

    def __init__(self):
        super().__init__("reset_counter") # criar um nó chamado "reset_counter"
        self.client_ = self.create_client(SetBool, "reset_counter") # criar um cliente de serviço chamado "reset_counter" que usa o tipo SetBool

    def call_reset_counter(self, value): # função para chamar o serviço de resetar o contador
        while not self.client_.wait_for_service(1.0): # esperar até que o serviço esteja disponível
            self.get_logger().warn("Waiting for Reset Counter server...") # registrar uma mensagem de log enquanto espera
        request = SetBool.Request() # criar uma requisição do tipo SetBool
        request.data = value # definir o valor da requisição

        future = self.client_.call_async(request) # chamar o serviço de forma assíncrona

        # adicionar uma função de callback para processar a resposta quando estiver pronta
        # se quiser passar argumentos adicionais para o callback, use partial
        future.add_done_callback(partial(self.callback_reset_counter, request=request))

    def callback_reset_counter(self, future, request): # função de callback que processa a resposta do serviço
        response = future.result()                # obter o resultado da resposta
        self.get_logger().info(f"Reset counter request: {request.data} | Success: {response.success} | Message: {response.message}")


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = ResetCounterClient() # instanciar o nó
    node.call_reset_counter(True) # chamar o serviço para resetar o contador
    node.call_reset_counter(False) # chamar o serviço para não resetar o contador
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()