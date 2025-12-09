#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServerNode(Node):

    def __init__(self):
        super().__init__("move_robot_server")       # criar um nó chamado "move_robot_server"
        self.goal_lock_ = threading.Lock()          # criar um bloqueio para gerenciar o acesso à meta atual
        self.goal_handle_ : ServerGoalHandle = None # armazenar o manipulador da meta atual
        self.robot_position_ = 50                   # posição inicial do robô
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            cancel_callback= self.cancel_callback,
            execute_callback= self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Action Server has been started!")         # registrar uma mensagem de log
        self.get_logger().info(f"Robot position: {self.robot_position_}") # registrar a posição inicial do robô

    # processar a meta
    def goal_callback(self, goal_request: MoveRobot.Goal): 
        self.get_logger().info("Receveid a new goal")

        # validar a posição e a velocidade do robô
        if goal_request.position not in range(0, 100) or goal_request.velocity <= 0:
            self.get_logger().warn("Invalid position/velocity, rejecting goal...")
            return GoalResponse.REJECT
        
        # New goal is valid, abort current goal and accept new goal
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.get_logger().info("Aborting current goal...")
            self.goal_handle_.abort()
        
        self.get_logger().info("Accepting goal")
        return GoalResponse.ACCEPT
    
    # processar o cancelamento da meta
    def cancel_callback(self, goal_handle: ServerGoalHandle): 
        self.get_logger().info("Received a cancel request...")
        # posição atual do robô:
        self.get_logger().info(f"Current robot position: {self.robot_position_}")
        return CancelResponse.ACCEPT

    # executar a ação para mover o robô
    def execute_callback(self, goal_handle: ServerGoalHandle): 

        with self.goal_lock_: # garantir que apenas uma meta seja processada por vez
            self.goal_handle_ = goal_handle 

        goal_position = goal_handle.request.position  # posição alvo do robô
        velocity = goal_handle.request.velocity       # velocidade de movimento do robô

        result = MoveRobot.Result()                   # criar um objeto para armazenar o resultado da ação
        feedback = MoveRobot.Feedback()               # criar um objeto para armazenar o feedback da ação

        self.get_logger().info("Execute goal")
        while rclpy.ok(): 
            if not goal_handle.is_active: # verificar se a meta ainda está ativa
                result.position = self.robot_position_
                result.message = "Preempted by another goal"
                return result
            
            if goal_handle.is_cancel_requested: # verificar se o cancelamento da meta foi solicitado
                result.position = self.robot_position_ # posição atual do robô
                # verificar se posição alvo foi alcançada
                if goal_position == self.robot_position_: 
                    result.message = "Success: Goal already achieved"
                    goal_handle.succeed()
                else:
                    result.message = "Goal canceled"
                    goal_handle.canceled()
                return result


            diff = goal_position - self.robot_position_ # diferença entre a posição alvo e a posição atual

            if diff == 0: # posição alvo alcançada
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result

            elif diff > 0: # mover para a direita
                if diff >= velocity: 
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff

            else: # mover para a esquerda
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)

            self.get_logger().info(f"Robot position: {self.robot_position_}")
            feedback.current_position = self.robot_position_ # atualizar a posição atual no feedback
            goal_handle.publish_feedback(feedback)           # publicar o feedback

            time.sleep(1.0)                                  # simular o tempo de movimento do robô


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MoveRobotServerNode() # instanciar o nó
    rclpy.spin(node, executor=MultiThreadedExecutor()) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()