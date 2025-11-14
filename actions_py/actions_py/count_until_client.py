#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilClientNode(Node):

    def __init__(self):
        super().__init__("count_until_client") # criar um nó chamado "count_until_client"

        # criar o cliente de ação
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until") 

    # enviar uma meta para o servidor de ação
    def send_goal(self, target_number, period): 
        # aguardar até que o servidor de ação esteja disponível(wait for the server)
        self.count_until_client_.wait_for_server()

        # criar uma meta(create a goal)
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # enviar a meta(send the goal)
        self.get_logger().info("Sending goal...")
        # adicionar um callback para processar a resposta do servidor de ação
        self.count_until_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 

    # processar a resposta do servidor de ação
    def goal_response_callback(self, future): 
        self.goal_handle_: ClientGoalHandle = future.result() # obter o manipulador da meta(goal handle)
        if self.goal_handle_.accepted:                        # verificar se a meta foi aceita
            self.get_logger().info("Goal got accepted!")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) # adicionar um callback para processar o resultado da meta
        else:
            self.get_logger().warn("Goal got rejected...")



    # processar o resultado da meta
    def goal_result_callback(self, future): 
        result = future.result().result                                 # obter o resultado da meta
        self.get_logger().info(f"Result: {str(result.reached_number)}") # exibir o número alcançado



def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = CountUntilClientNode() # instanciar o nó
    node.send_goal(6, 1.0) # testar o envio de uma meta: contar até 6 com período de 1 segundo
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()