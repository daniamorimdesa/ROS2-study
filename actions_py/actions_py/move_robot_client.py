#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from example_interfaces.msg import Empty

class MoveRobotClientNode(Node):

    def __init__(self):
        super().__init__("move_robot_client") # criar um nó chamado "move_robot_client"
        self.goal_handle_ = None               # armazenar o manipulador da meta atual
        self.move_robot_client_ = ActionClient(self, MoveRobot, "move_robot")
        self.cancel_subscriber = self.create_subscription(Empty, "cancel_move", self.callback_cancel_move, 10) # criar um assinante para o tópico "cancel_move"

    # enviar uma meta para o servidor de ação
    def send_goal(self, position, velocity): 
        self.move_robot_client_.wait_for_server() # aguardar até que o servidor de ação esteja disponível

        goal = MoveRobot.Goal()   # criar um objeto de meta
        goal.position = position  # definir a posição alvo do robô
        goal.velocity = velocity  # definir a velocidade de movimento do robô

        self.get_logger().info(f"Send goal with position {str(position)} and velocity {str(velocity)}")

        # enviar a meta de forma assíncrona
        self.move_robot_client_.send_goal_async( 
            goal,
            feedback_callback=self.goal_feedback_callback).add_done_callback( # registrar a função de callback para feedback
            self.goal_response_callback                                       # registrar a função de callback para a resposta da meta
        )

    # processar a resposta da meta
    def goal_response_callback(self, future): 
        self.goal_handle_: ClientGoalHandle = future.result() # obter o manipulador da meta a partir do futuro
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted!")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) # registrar a função de callback para o resultado da meta
        else:
            self.get_logger().info("Goal got rejected!")

    # processar o resultado da meta
    def goal_result_callback(self, future): 
        status = future.result().status          # obter o status do resultado da meta
        result = future.result().result          # obter o resultado da meta
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Position: {str(result.position)}")  # registrar a posição final do robô
        self.get_logger().info(f"Message: {str(result.message)}")    # registrar a mensagem do resultado

    # processar o feedback da meta
    def goal_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position             # obter a posição atual do feedback
        self.get_logger().info(f"Feedback position: {str(position)}") # registrar a posição atual do feedback

    def callback_cancel_move(self, msg):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Send a cancel request")
            self.goal_handle_.cancel_goal_async() # enviar uma solicitação de cancelamento

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = MoveRobotClientNode() # instanciar o nó
    node.send_goal(76,1)
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()