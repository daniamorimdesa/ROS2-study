#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot

class MoveRobotServerNode(Node):

    def __init__(self):
        super().__init__("move_robot_server") # criar um nó chamado "move_robot_server"
        self.robot_position_ = 50             # posição inicial do robô
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            execute_callback= self.execute_callback
        )
        self.get_logger().info("Action Server has been started!")         # registrar uma mensagem de log
        self.get_logger().info(f"Robot position: {self.robot_position_}") # registrar a posição inicial do robô

    # executar a ação para mover o robô
    def execute_callback(self, goal_handle: ServerGoalHandle): 
        goal_position = goal_handle.request.position  # posição alvo do robô
        velocity = goal_handle.request.velocity       # velocidade de movimento do robô

        result = MoveRobot.Result()                   # criar um objeto para armazenar o resultado da ação
        feedback = MoveRobot.Feedback()               # criar um objeto para armazenar o feedback da ação

        self.get_logger().info("Execute goal")
        while rclpy.ok(): 
            diff = goal_position - self.robot_position_ # diferença entre a posição alvo e a posição atual

            if diff == 0: # posição alvo alcançada
                result.position = self.robot_position_
                result.message = "Sucess"
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
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()