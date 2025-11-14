#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node 
from rclpy.action import ActionServer, GoalResponse 
from rclpy.action.server import ServerGoalHandle 
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):

    def __init__(self):
        super().__init__("count_until_server")      # criar um nó chamado "count_until_server"
        self.count_until_server_ = ActionServer(    # criar o servidor de ação 
            self,                                   # nó atual
            CountUntil,                             # tipo de ação
            "count_until",                          # nome da ação
            goal_callback=self.goal_callback,       # função de callback para processar a meta
            execute_callback=self.execute_callback) # função de callback para executar a ação
        
        # log de inicialização do servidor de ação
        self.get_logger().info("Count Until Action Server has been started!") 


    # função de callback para processar a meta
    def goal_callback(self, goal_request: CountUntil.Goal): 
        self.get_logger().info("Received a goal") # log de recebimento da meta

        # validar o goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal...") # log de rejeição da meta
            return GoalResponse.REJECT                   # rejeitar metas com número alvo menor ou igual a zero
        
        self.get_logger().info("Accepting the goal!") # log de aceitação da meta
        return GoalResponse.ACCEPT                   # aceitar a meta


    
    # função de callback para executar a ação
    def execute_callback(self, goal_handle: ServerGoalHandle): 

        # pegar a requisição da meta(get request from the goal)
        target_number = goal_handle.request.target_number # obter o número alvo da meta
        period = goal_handle.request.period               # obter o período entre contagens

        # executar a ação(execute the action)
        self.get_logger().info("Executing the goal...")           # log de execução da meta
        counter = 0                                            # inicializar o contador
        for i in range(target_number):                         # loop até o número alvo
            counter+=1                                         # incrementar o contador
            self.get_logger().info(f"counter: {str(counter)}") # log do contador atual
            time.sleep(period)                                 # esperar pelo período especificado

        # uma vez que o contador atinge o número alvo, definir o estado final(once done, set the final state)
        goal_handle.succeed()                                 # marcar a meta como sucedida

        # mandar o resultado(send the result)
        result = CountUntil.Result()                         # criar o resultado
        result.reached_number = counter                      # definir o número alcançado no resultado
        return result                                        # retornar o resultado


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = CountUntilServerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()