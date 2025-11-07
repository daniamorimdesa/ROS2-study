#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn  # importar o serviço Spawn do turtlesim
from turtlesim.srv import Kill  # importar o serviço Kill do turtlesim
from my_robot_interfaces.msg import TurtleArray, Turtle  # importar as mensagens personalizadas TurtleArray e Turtle
from my_robot_interfaces.srv import CatchTurtle  # importar o serviço personalizado CatchTurtle
from functools import partial
from math import pi
from random import uniform

class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("turtle_spawner") # criar um nó chamado "turtle_spawner"

        self.turtle_name_prefix_ = "turtle"
        self.turtle_counter_ = 1 # contador de tartarugas spawnadas
        self.alive_turtles_ = [] # lista de tartarugas vivas

        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10) # criar um publisher para as tartarugas vivas

        self.spawn_client_ = self.create_client(Spawn, "/spawn")                 # criar um cliente para o serviço /spawn
        self.spawn_turtle_timer_ = self.create_timer(2.0, self.spawn_new_turtle) # criar um timer para spawnar tartarugas a cada 2 segundos


    def publish_alive_turtles(self): # publicar a lista de tartarugas vivas
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)


    def spawn_new_turtle(self): # callback para spawnar uma nova tartaruga
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_) # criar um nome único para a tartaruga
        x = uniform(2.0, 10.0)                                      # escolher uma coordenada x aleatória entre 2.0 e 10.0
        y = uniform(2.0, 10.0)                                      # escolher uma coordenada y aleatória entre 2.0 e 10.0
        theta = uniform(0.0, 2*pi)                                  # escolher uma orientação theta aleatória entre 0.0 e 2*pi
        self.call_spawn_service(name, x, y, theta)                  # chamar o serviço de spawnar a tartaruga

  
    def call_spawn_service(self, turtle_name, x, y, theta): # chamar o serviço de spawnar a tartaruga
        while not self.spawn_client_.wait_for_service(1.0): # esperar até que o serviço esteja disponível
            self.get_logger().warn("Waiting for spawn service...")
        
        request = Spawn.Request()                        # criar uma requisição para o serviço Spawn
        request.x = x                                    # definir a coordenada x
        request.y = y                                    # definir a coordenada y
        request.theta = theta                            # definir a orientação theta
        request.name = turtle_name                       # definir o nome da tartaruga

        future = self.spawn_client_.call_async(request)                                      # chamar o serviço de spawn de forma assíncrona
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request)) # adicionar um callback para processar a resposta do serviço

    def callback_call_spawn_service(self, future, request: Spawn.Request):
        response = Spawn.Response = future.result()                        # obter a resposta do serviço
        if response.name != "":                                            # se a tartaruga foi spawnada com sucesso
            self.get_logger().info(f"New alive turtle: {response.name}")

            # adicionar a nova tartaruga à lista de vivas
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()  # publicar a lista atualizada de tartarugas vivas


    
def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleSpawnerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()