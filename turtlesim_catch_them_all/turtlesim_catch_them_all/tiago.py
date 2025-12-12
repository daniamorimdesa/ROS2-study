import rclpy
import time
from rclpy.node import Node
# from turtlesim.msg import Pose

from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi

"""
Adaptar para o robô TIAgo, que deve se mover para uma posição desejada, 
publicando no tópico /cmd_vel. O robô deve rotacionar até alinhar com a 
direção do destino e depois se mover em linha reta até chegar na posição
desejada. 
"""

class TurtlePoseNode(Node):
    def __init__(self):
        super().__init__("turtle_pose_node") # criar um nó chamado "turtle_controller"

        self.pose_ = None

        self.twist_cmd = Twist()

        self.rotationIsOver = False # flag para indicar se a rotação foi concluída
        self.goalReached = False    # flag para indicar se chegou no destino

        #criar um subscriber pro /turtle1/pose para saber a posição atual da turtle1
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

        # criar um publisher na /turtle1/cmd_vel para mover a turtle1
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("Turtle Pose Node has been started!") # registrar uma mensagem de log


    def callback_pose(self, pose: Pose): 

        # verificar se chegou no destino
        if self.goalReached:
            return

        # posição final desejada
        x_goal = 2.0
        y_goal = 3.0

        cmd = Twist() # criar uma nova mensagem Twist
        K_angular = 2.0 
        K_linear = 1.0

        # calcular distâncias
        dist_x = x_goal - pose.x
        dist_y = y_goal - pose.y
        distance = sqrt(dist_x**2 + dist_y**2)

        # calcular ângulo desejado
        goal_theta = atan2(dist_y, dist_x) 
        diff_angular = goal_theta - pose.theta 
        
        # normalizar ângulo para [-pi, pi]
        if diff_angular > pi:
            diff_angular -= 2 * pi
        elif diff_angular < -pi:
            diff_angular += 2 * pi

        # Rotacionar até alinhar
        if not self.rotationIsOver:
            cmd.linear.x = 0.0
            cmd.angular.z = K_angular * diff_angular
            
            if abs(diff_angular) < 0.001: # se a diferença angular for pequena o suficiente
                self.rotationIsOver = True
                cmd.angular.z = 0.0
                self.get_logger().info("Rotation complete!")
        
        # Mover em linha reta
        elif distance > 0.01:
            cmd.linear.x = K_linear * distance

        # quando chegar no destino
        else:
            self.goalReached = True
            self.get_logger().info(f"Goal reached! x={pose.x:.2f}, y={pose.y:.2f}")
            return
        
        self.cmd_vel_publisher_.publish(cmd) # publicar o comando de velocidade



    # testar movimento linear
    def test_twist(self): 
        # começar
        self.twist_cmd.linear.x = 0.2
        self.cmd_vel_publisher_.publish(self.twist_cmd)

        time.sleep(1)

        # parar
        self.twist_cmd.linear.x = 0.0
        self.cmd_vel_publisher_.publish(self.twist_cmd)


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtlePoseNode() # instanciar o nó
    # node.test_twist()
    rclpy.spin(node) # manter o nó ativo
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()

