#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

"""node para gerenciar o ciclo de vida de outro node
será usado para gerenciar o node number_publisher do pacote lifecycle_py

Para rodar esse node:
ros2 run lifecycle_py lifecycle_node_manager --ros-args -p managed_node_name:="number_publisher"

em que "number_publisher" é o nome do node gerenciado
"""

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")                                     # criar um nó chamado "lifecycle_manager"
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)  # declarar o parâmetro para o nome do nó gerenciado
        node_name = self.get_parameter("managed_node_name").value                 # obter o valor do parâmetro
        service_change_state_name = "/" + node_name + "/change_state"             # construir o nome do serviço de mudança de estado
        self.client = self.create_client(ChangeState, service_change_state_name)  # criar o cliente do serviço de mudança de estado
    
    # enviar a requisição de mudança de estado
    def change_state(self, transition: Transition): 
        self.client.wait_for_service()                          # esperar o serviço estar disponível
        request = ChangeState.Request()                         # criar a requisição
        request.transition = transition                         # definir a transição desejada
        future = self.client.call_async(request)                # chamar o serviço de forma assíncrona
        rclpy.spin_until_future_complete(self, future)          # esperar a resposta
   
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
    node = LifecycleNodeManager()         # criar uma instância do nó LifecycleNodeManager
    node.initialization_sequence()        # executar a sequência de inicialização
    rclpy.shutdown()                      # desligar o rclpy

#------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
