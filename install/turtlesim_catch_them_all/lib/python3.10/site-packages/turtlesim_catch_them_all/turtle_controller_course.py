#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
# from math import atan2, pi
# from my_robot_interfaces.msg import TurtleArray
# from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # criar um nó chamado "turtle_controller"

        self.pose_: Pose = None 

        #criar um subscriber pro /turtle1/pose para saber a posição atual da turtle1
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

    def callback_pose(self, pose: Pose):
        self.pose_ = pose # armazenar a posição atual da turtle1
        

def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleControllerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()