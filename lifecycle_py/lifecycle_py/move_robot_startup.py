#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

"""node para gerenciar o ciclo de vida de outro node
- será usado para gerenciar o node move_robot_server do pacote lifecycle_py, gerenciando múltiplos servidores de ação
- para rodar esse node com dois servidores de ação:
ros2 run lifecycle_py move_robot_startup --ros-args -p managed_node_names:="['move_robot_server_a', 'move_robot_server_b']"

em que "move_robot_server" é o nome do node gerenciado
"""

class MoveRobotStartUp(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")                                            # criar um nó chamado "lifecycle_manager"
        self.declare_parameter("managed_node_names", rclpy.Parameter.Type.STRING_ARRAY)  # declarar o parâmetro para o nome do nó gerenciado
        node_name_list = self.get_parameter("managed_node_names").value                  # obter o valor do parâmetro

        # imprimir nomes
        self.get_logger().info(f"Managed node names: {node_name_list}")

        self.client_list = []
        for node_name in node_name_list:
            service_change_state_name = "/" + node_name + "/change_state"                # construir o nome do serviço de mudança de estado
            client = self.create_client(ChangeState, service_change_state_name)          # criar o cliente do serviço de mudança de estado
            self.client_list.append((node_name, client))                                 # armazenar o cliente na lista com o nome do nó


    # enviar a requisição de mudança de estado
    def change_state(self, transition: Transition): 
        for node_name, client in self.client_list:
            client.wait_for_service()                          # esperar o serviço estar disponível
            request = ChangeState.Request()                    # criar a requisição
            request.transition = transition                    # definir a transição desejada
            future = client.call_async(request)                # chamar o serviço de forma assíncrona
            rclpy.spin_until_future_complete(self, future)     # esperar a resposta
   
    # realizar a sequência de inicialização do nó gerenciado
    def initialization_sequence(self): 
        # Unconfigured to Inactive (configure)
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()                           # criar a transição
        transition.id = Transition.TRANSITION_CONFIGURE     # definir a transição para configurar
        transition.label = "configure"                      # definir o rótulo da transição
        self.change_state(transition)                       # enviar a requisição de mudança de estado
        self.get_logger().info("Configuring OK, now inactive") 

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active (activate)
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()                           # criar a transição
        transition.id = Transition.TRANSITION_ACTIVATE      # definir a transição para ativar
        transition.label = "activate"                       # definir o rótulo da transição
        self.change_state(transition)                       # enviar a requisição de mudança de estado
        self.get_logger().info("Activating OK, now active")


def main(args=None):
    rclpy.init(args=args)                 # inicializar o rclpy
    node = MoveRobotStartUp()             # criar uma instância do nó MoveRobotStartUp
    node.initialization_sequence()        # executar a sequência de inicialização
    rclpy.shutdown()                      # desligar o rclpy

#------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
