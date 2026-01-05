#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces_exercise_2.srv import CellOccupancy # importar o serviço CellOccupancy
from functools import partial

"""
Este node implementa um cliente de serviço ROS2 chamado "cell_occupancy_client" que envia requisições
para verificar se uma célula do mapa(grid 8x8) está ocupada ou livre. 
O cliente utiliza o serviço CellOccupancy, que inclui os campos:
- Request(coordenadas de uma célula): x (int), y (int)
- Response(livre ou ocupada): occupied (bool) | message (string)
"""

class CellOccupancyClient(Node):

    def __init__(self):
        super().__init__("cell_occupancy_client") # criar um nó chamado "cell_occupancy_client"

        # criar um cliente de serviço chamado "cell_occupancy" que usa o tipo CellOccupancy
        self.client_ = self.create_client(CellOccupancy, "cell_occupancy") 

        self.get_logger().info('\033[92mCell Occupancy client has been started!\033[0m') # log verde
        
    # função para chamar o serviço de verificar ocupação
    def call_check_occupancy(self, x, y): 

        # esperar até que o serviço esteja disponível
        while not self.client_.wait_for_service(1.0): 
            self.get_logger().warn("Waiting for Cell Occupancy server...") 

        # criar uma requisição do tipo CellOccupancy
        request = CellOccupancy.Request() 
        
        # definir os valores dos campos na requisição
        request.x = x
        request.y = y

        # chamar o serviço de forma assíncrona
        future = self.client_.call_async(request) 
        
        # adicionar uma função de callback para processar a resposta quando estiver pronta
        future.add_done_callback(partial(self.callback_call_check_occupancy, request=request))

    # função de callback que processa a resposta do serviço
    def callback_call_check_occupancy(self, future, request): 

        # obter o resultado da resposta
        response = future.result()  

        # registrar a resposta recebida
        if response.occupied:
            self.get_logger().info(f"Requested cell: ({request.x}, {request.y}) | Occupied: {response.occupied} | Message: {response.message}") 
        else:
            self.get_logger().info(f'\033[96mRequested cell: ({request.x}, {request.y}) | Occupied: {response.occupied} | Message: {response.message}\033[0m') 
    



def main(args=None):
    rclpy.init(args=args)                  
    node = CellOccupancyClient()                

    # chamar o serviço para verificar ocupação
    node.call_check_occupancy(6, 1)    
    node.call_check_occupancy(9, 3) 
    node.call_check_occupancy(3, 3)
    node.call_check_occupancy(0, 0)   
    node.call_check_occupancy(7, 7)   
   

    rclpy.spin(node)                       
    rclpy.shutdown()                      
#------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()