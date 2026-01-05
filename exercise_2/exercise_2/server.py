#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces_exercise_2.srv import CellOccupancy # importar o serviço CellOccupancy

"""
Este node implementa um servidor de serviço ROS2 que indica se uma célula 
do mapa(grid 8x8) está ocupada ou livre. 
- Request(coordenadas de uma célula): x (int), y (int)
- Response(livre ou ocupada): occupied (bool) | message (string)

O mapa será armazenado no serviço como uma matriz 8x8 interna, 
onde 0 indica célula livre e 1 célula ocupada.
"""

class CellOccupancyServer(Node):

    def __init__(self):
        super().__init__("cell_occupancy_server") 

        # criar um servidor de serviço chamado "cell_occupancy" que usa o tipo cell_occupancy 
        # e chama callback_verify_occupancy quando uma requisição é recebida
        self.server_ = self.create_service(CellOccupancy, "cell_occupancy", self.callback_verify_occupancy) 

        # definir o mapa 8x8 (0 = livre, 1 = ocupado)

        # células ocupadas (x representa coluna, y representa linha):
        # x=6, y=1
        # x=6, y=2
        # x=0, y=3
        # x=1, y=3
        # x=2, y=3
        # x=3, y=3
        # x=6, y=3
        # x=3, y=4
        # x=6, y=4
        # x=3, y=5
        # x=4, y=5
        # x=5, y=5
        # x=6, y=5

        self.map_ = [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [1, 1, 1, 1, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]

        self.get_logger().info('\033[92mCell Occupancy server has been started!\033[0m')

    # função de callback que processa a requisição e retorna a resposta
    def callback_verify_occupancy(self, request: CellOccupancy.Request, response: CellOccupancy.Response): 

        # verificar se as coordenadas estão dentro dos limites do mapa (0 a 7)
        if 0 <= request.x < 8 and 0 <= request.y < 8:
            cell_value = self.map_[request.y][request.x]
            response.occupied = bool(cell_value)
            if response.occupied:
                response.message = "Requested cell is occupied!"
                self.get_logger().warn(response.message)
            else:
                response.message = "Requested cell is free :)"
                self.get_logger().info(f'\033[96m{response.message}\033[0m') 
            return response

        # caso as coordenadas estejam fora dos limites
        response.occupied = False # assumir livre se fora dos limites
        response.message = "Requested coordinates are out of map bounds!"
        self.get_logger().info(f'\033[95m{response.message}\033[0m')
        return response


def main(args=None):
    rclpy.init(args=args)     
    node = CellOccupancyServer()   
    rclpy.spin(node)         
    rclpy.shutdown()          
#-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()