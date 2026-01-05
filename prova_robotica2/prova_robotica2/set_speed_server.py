#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces_prova_robotica2.srv import SetSpeed # importar o serviço SetSpeed

"""
Este node implementa um servidor de serviço ROS2 chamado "set_speed_server" que processa 
requisições para definir a velocidade linear e angular de um robô.
O servidor aceita requisições do tipo SetSpeed, que inclui os campos:
- enable (bool): indica se o robô deve ser habilitado
- linear (float): velocidade linear desejada
- angular (float): velocidade angular desejada
"""



class SetSpeedServer(Node):

    def __init__(self):
        super().__init__("set_speed_server") # criar um nó chamado "set_speed_server"

        # criar um servidor de serviço chamado "set_speed" que usa o tipo SetSpeed 
        # e chama callback_set_speed quando uma requisição é recebida
        self.server_ = self.create_service(SetSpeed, "set_speed", self.callback_set_speed) 

        # estado interno
        self.enabled_ = False # indica se o robô está habilitado
        self.linear_ = 0.0    # velocidade linear
        self.angular_ = 0.0   # velocidade angular

        self.get_logger().info('\033[92mSet Speed server has been started!\033[0m') # log verde

    # função de callback que processa a requisição e retorna a resposta
    def callback_set_speed(self, request: SetSpeed.Request, response: SetSpeed.Response): 

        # atualizar o estado de habilitação
        self.enabled_ = request.enable 

        # definir limites máximos de velocidade 
        max_linear = 1.0
        max_angular = 2.0

        # verificar se o robô está habilitado
        if not self.enabled_:
            self.linear_ = 0.0
            self.angular_ = 0.0
            response.success = True
            response.message = "Robot disabled. Speeds set to zero."
            self.get_logger().info(f'\033[95m{response.message}\033[0m') # log cor magenta
            response.applied_linear = self.linear_
            response.applied_angular = self.angular_
            return response

        # robô está habilitado, atualizar velocidades
        self.linear_ = request.linear   # atualizar a velocidade linear
        self.angular_ = request.angular # atualizar a velocidade angular

        # verificar se as velocidades solicitadas estão dentro dos limites
        if abs(self.linear_) > max_linear or abs(self.angular_) > max_angular:
            response.success = False
            response.message = f"Invalid speeds. Limits: |linear|<= {max_linear}, |angular|<= {max_angular}"
            self.get_logger().warn(response.message)
            response.applied_linear = 0.0
            response.applied_angular = 0.0
            self.linear_ = 0.0
            self.angular_ = 0.0
        else:
            response.success = True
            response.message = "Speeds set successfully!"
            self.get_logger().info(f'\033[96m{response.message}\033[0m') # log cor ciano
            self.get_logger().info(f'\033[94menable={self.enabled_}|linear={self.linear_:.2f}|angular={self.angular_:.2f}\033[0m') # log cor azul
            response.applied_linear = self.linear_
            response.applied_angular = self.angular_

        return response # retornar a resposta


def main(args=None):
    rclpy.init(args=args)     # inicializar o rclpy
    node = SetSpeedServer()   # instanciar o nó
    rclpy.spin(node)          # manter o nó ativo para processar callbacks
    rclpy.shutdown()          # finalizar o rclpy
#-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()