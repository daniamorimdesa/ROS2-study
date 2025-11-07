#!/usr/bin/env python3
from functools import partial
from random import uniform
from math import pi
import uuid
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn  # importar o serviço Spawn do turtlesim
from turtlesim.srv import Kill  # importar o serviço Kill do turtlesim
from my_robot_interfaces.msg import TurtleArray, Turtle  # importar as mensagens personalizadas TurtleArray e Turtle
from my_robot_interfaces.srv import CatchTurtle  # importar o serviço personalizado CatchTurtle

"""
- Chamar o serviço de /spawn para criar tartarugas na tela do turtlesim(escolher coordenadas aleatórias entre 0.0 e 11.0 para x e y).
- Publicar a lista de tartarugas vivas no tópico /alive_turtles usando a mensagem TurtleArray personalizada.
- Implementar o serviço /catch_turtle que recebe o nome de uma tartaruga e chama o serviço /kill para removê-la da tela do turtlesim e
atualiza a lista de tartarugas vivas.
"""


class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("turtle_spawner") # criar um nó chamado "turtle_spawner"

        self.turtle_name_prefix_ = "turtle"
        self.turtle_counter_ = 1 # contador de tartarugas spawnadas
        self.alive_turtles_ = [] # lista de tartarugas vivas

        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10) # criar um publisher para as tartarugas vivas

        self.spawn_client_ = self.create_client(Spawn, "/spawn")   # criar um cliente para o serviço /spawn
        self.kill_client_ = self.create_client(Kill, "/kill")      # criar um cliente para o serviço /kill

        # criar um serviço para pegar tartarugas
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle) 

        # criar um timer para spawnar tartarugas a cada 0.5 segundos
        self.spawn_turtle_timer_ = self.create_timer(0.5, self.spawn_new_turtle)

        self.get_logger().info("Turtle Spawner has been started!") # registrar uma mensagem de log

    
    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        # call kill service
        self.call_kill_service(request.name)
        response.success = True
        return response


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
        response: Spawn.Response = future.result()                        # obter a resposta do serviço
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
    
    def call_kill_service(self, turtle_name): # chamar o serviço /kill
        while not self.kill_client_.wait_for_service(1.0): # esperar até que o serviço esteja disponível
            self.get_logger().warn("Waiting for kill service...")

        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name=turtle_name))

    def callback_call_kill_service(self, future, turtle_name):
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == turtle_name:
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                break


    
def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleSpawnerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()