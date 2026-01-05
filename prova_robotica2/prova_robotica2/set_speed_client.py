#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces_prova_robotica2.srv import SetSpeed # importar o serviço SetSpeed
from functools import partial

"""
Este node implementa um cliente de serviço ROS2 chamado "set_speed_client" que envia requisições
para definir a velocidade linear e angular de um robô. 
O cliente utiliza o serviço SetSpeed, que inclui os campos:
- enable (bool): indica se o robô deve ser habilitado
- linear (float): velocidade linear desejada
- angular (float): velocidade angular desejada
"""
class SetSpeedClient(Node):

    def __init__(self):
        super().__init__("set_speed_client") # criar um nó chamado "set_speed_client"

        # criar um cliente de serviço chamado "set_speed" que usa o tipo SetSpeed
        self.client_ = self.create_client(SetSpeed, "set_speed") 

        self.get_logger().info('\033[92mSet Speed client has been started!\033[0m') # log verde

    # função para chamar o serviço de definir velocidade
    def call_set_speed(self, enable, linear, angular): 

        # esperar até que o serviço esteja disponível
        while not self.client_.wait_for_service(1.0): 
            self.get_logger().warn("Waiting for Set Speed server...") 

        # criar uma requisição do tipo SetSpeed
        request = SetSpeed.Request() 
        
        # definir os valores dos campos na requisição
        request.enable = enable
        request.linear = linear
        request.angular = angular

        # chamar o serviço de forma assíncrona
        future = self.client_.call_async(request) 
        
        # adicionar uma função de callback para processar a resposta quando estiver pronta
        #future.add_done_callback(self.callback_call_set_speed) 
        # se quiser passar argumentos adicionais para o callback, use partial
        future.add_done_callback(partial(self.callback_call_set_speed, request=request))

    # função de callback que processa a resposta do serviço
    def callback_call_set_speed(self, future, request): 

        # obter o resultado da resposta
        response = future.result()  

        # registrar a resposta recebida
        self.get_logger().info(f'\033[36mGot response: {str(response.message)}\033[0m') # cor ciano

        # registrar a resposta com os valores da requisição (partial serve para passar a requisição original)
        # self.get_logger().info(
        #     f'\033[36mRequest -> Enable: {request.enable}, Linear: {request.linear}, Angular: {request.angular} | '
        #     f'Response -> Success: {response.success}, Applied Linear: {response.applied_linear}, Applied Angular: {response.applied_angular}\033[0m'
        # ) # cor ciano



def main(args=None):
    rclpy.init(args=args)                  # inicializar o rclpy
    node = SetSpeedClient()                # instanciar o nó

    # chamar o serviço para definir velocidade
    node.call_set_speed(True, 1.0, 0.5)    
    node.call_set_speed(False, 0.0, 0.0)   
    node.call_set_speed(True, 2.0, 1.0)    

    rclpy.spin(node)                       # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown()                       # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()