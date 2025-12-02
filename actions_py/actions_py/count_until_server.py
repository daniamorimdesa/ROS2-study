#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle 
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node):

    def __init__(self):
        super().__init__("count_until_server")                       # criar um nó chamado "count_until_server"
        self.goal_handle_ : ServerGoalHandle = None                  # inicializar o manipulador da meta(goal handle) como None
        self.goal_lock_ = threading.Lock()                           # criar um lock para proteger o acesso ao goal_handle_
        self.goal_queue_ = []                                        # fila para armazenar metas recebidas
        self.count_until_server_ = ActionServer(                     # criar o servidor de ação 
            self,                                                    # nó atual
            CountUntil,                                              # tipo de ação
            "count_until",                                           # nome da ação
            goal_callback=self.goal_callback,                        # função de callback para processar a meta
            handle_accepted_callback=self.handle_accepted_callback,  # função de callback para quando uma meta é aceita
            cancel_callback=self.cancel_callback,                    # função de callback para cancelar a meta
            execute_callback=self.execute_callback,                  # função de callback para executar a ação
            callback_group=ReentrantCallbackGroup())                 # permitir callbacks concorrentes
        
        # log de inicialização do servidor de ação
        self.get_logger().info("Count Until Action Server has been started!") 


    # função de callback para processar a meta
    def goal_callback(self, goal_request: CountUntil.Goal): 
        self.get_logger().info("Received a goal") # log de recebimento da meta

        # # Policy: recusar metas se a meta atual estiver ativa(goal still active)
        # with self.goal_lock_:                                                          # proteger o acesso ao goal_handle_ com um lock
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting new goal") # log de rejeição da meta
        #         return GoalResponse.REJECT                                             # rejeitar a meta se outra estiver ativa

        # validar o goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal...") # log de rejeição da meta
            return GoalResponse.REJECT                      # rejeitar metas com número alvo menor ou igual a zero
        
        # # Policy: preempt existing goal when receiving a new goal(abortar a meta atual ao receber uma nova)
        # with self.goal_lock_:                                                    # proteger o acesso ao goal_handle_ com um lock
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accept new goal") # log de preempção da meta
        #         self.goal_handle_.abort()                                        # marcar a meta atual como cancelada
        

        self.get_logger().info("Accepting the goal!") # log de aceitação da meta
        return GoalResponse.ACCEPT                    # aceitar a meta

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:                          # proteger o acesso ao goal_handle_ com um lock
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)   # adicionar a nova meta à fila
            else:
                goal_handle.execute()                  # iniciar a execução da meta se nenhuma outra estiver ativa

    # função de callback para cancelar a meta
    def cancel_callback(self, goal_handle: ServerGoalHandle): 
        self.get_logger().info("Received a cancel request") # log de recebimento do pedido de cancelamento
        return CancelResponse.ACCEPT # or REJECT            # aceitar o pedido de cancelamento
    
    # função de callback para executar a ação
    def execute_callback(self, goal_handle: ServerGoalHandle): 
        with self.goal_lock_:                          # proteger o acesso ao goal_handle_ com um lock
            self.goal_handle_ = goal_handle            # armazenar o manipulador da meta(goal handle)

        # pegar a requisição da meta(get request from the goal)
        target_number = goal_handle.request.target_number # obter o número alvo da meta
        feedback = CountUntil.Feedback()
        period = goal_handle.request.period               # obter o período entre contagens

        # executar a ação(execute the action)
        self.get_logger().info("Executing the goal...")        # log de execução da meta
        result = CountUntil.Result()                           # criar o resultado
        counter = 0                                            # inicializar o contador

        # loop até o número alvo
        for i in range(target_number):   
            if not goal_handle.is_active:                            # verificar se a meta ainda está ativa
                result.reached_number = counter                      # definir o número alcançado no resultado
                self.process_next_goal_in_queue()                    # processar a próxima meta na fila
                return result                                        # retornar o resultado até o ponto de inatividade       
                                 
            if goal_handle.is_cancel_requested:                      # verificar se o cancelamento foi solicitado
                self.get_logger().info("Canceling the goal...")      # log de cancelamento da meta
                goal_handle.canceled()                               # marcar a meta como cancelada
                result.reached_number = counter                      # definir o número alcançado no resultado
                self.process_next_goal_in_queue()                    # processar a próxima meta na fila
                return result                                        # retornar o resultado até o ponto de cancelamento                                               
            
            counter+=1                                         # incrementar o contador
            self.get_logger().info(f"counter: {str(counter)}") # log do contador atual
            feedback.current_number = counter                  # definir o número atual no feedback
            goal_handle.publish_feedback(feedback)             # publicar o feedback
            time.sleep(period)                                 # esperar pelo período especificado

        # uma vez que o contador atinge o número alvo, definir o estado final(once done, set the final state)
        goal_handle.succeed()   # marcar a meta como sucedida
        # goal_handle.abort()   # marcar a meta como abortada

        # mandar o resultado(send the result)
        
        result.reached_number = counter                      # definir o número alcançado no resultado
        self.process_next_goal_in_queue()                    # processar a próxima meta na fila
        return result                                        # retornar o resultado
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:                               # proteger o acesso ao goal_handle_ com um lock
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()           # iniciar a execução da próxima meta na fila
            else:
                self.goal_handle_ = None                    # resetar o manipulador da meta se a fila estiver vazia

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = CountUntilServerNode() # instanciar o nó
    rclpy.spin(node, MultiThreadedExecutor()) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()