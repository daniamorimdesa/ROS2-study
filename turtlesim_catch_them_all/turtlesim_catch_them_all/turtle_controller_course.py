#!/usr/bin/env python3
from math import sqrt, atan2, pi
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # criar um nó chamado "turtle_controller"

        self.turtle_to_catch_ = None # tartaruga alvo para pegar
        self.pose_: Pose = None 

        #criar um subscriber pro /turtle1/pose para saber a posição atual da turtle1
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

        # criar um subscriber pro /alive_turtles para saber a lista de tartarugas vivas
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)

        # criar um publisher na /turtle1/cmd_vel para mover a turtle1
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # criar um timer para o loop de controle
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

        # criar um cliente para o serviço /catch_turtle
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")

        self.get_logger().info("Turtle Controller has been started!") # registrar uma mensagem de log


    def callback_pose(self, pose: Pose): 
        self.pose_ = pose # armazenar a posição atual da turtle1

    def callback_alive_turtles(self, msg: TurtleArray): # processar a lista de tartarugas vivas recebida
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ is None:
            return

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = sqrt(dist_x*dist_x + dist_y*dist_y)

        goal_theta = atan2(dist_y, dist_x)
        diff = goal_theta - self.pose_.theta

        # controlador proporcional simples
        K_linear = 2.0 
        K_angular = 6.0

        cmd = Twist()

        if distance > 0.5:
            # position
            cmd.linear.x = K_linear*distance

            # orientation
            if diff > pi:
                diff -= 2*pi
            elif diff < -pi:
                diff += 2*pi

            cmd.angular.z = K_angular*diff
             
        else:
            # target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # chamar o serviço de pegar tartaruga quando chegar perto o suficiente
            self.call_catch_turtle_service(self.turtle_to_catch_.name) 
            self.turtle_to_catch_ : Turtle = None  # resetar a tartaruga alvo após tentar pegar

        self.cmd_vel_publisher_.publish(cmd)

    def call_catch_turtle_service(self, turtle_name): # chamar o serviço de pegar tartaruga
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for catch turtle service... ")

        request = CatchTurtle.Request() # criar uma requisição para o serviço CatchTurtle
        request.name = turtle_name      # definir o nome da tartaruga a ser pega

        future = self.catch_turtle_client_.call_async(request)                                              # chamar o serviço de pegar tartaruga de forma assíncrona
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name)) # adicionar um callback para processar a resposta do serviço

    def callback_call_catch_turtle_service(self, future, turtle_name): # processar a resposta do serviço de pegar tartaruga
        response = future.result()  # obter a resposta do serviço
        if response.success:
            self.get_logger().info(f"Turtle {turtle_name} caught successfully!")
        else:
            self.get_logger().info(f"Failed to catch turtle {turtle_name}.")


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleControllerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()