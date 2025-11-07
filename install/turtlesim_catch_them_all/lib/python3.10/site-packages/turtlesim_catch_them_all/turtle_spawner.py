#!/usr/bin/env python3
from functools import partial
import random
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

        self.alive_turtles_ = []
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "/alive_turtles", 10)

        # clients criados uma vez
        self.spawn_turtle_client_ = self.create_client(Spawn, "/spawn")
        self.kill_turtle_client_ = self.create_client(Kill, "/kill")

        self.spawn_turtle_timer_ = self.create_timer(1.0, self.spawn_turtle_callback)
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "/catch_turtle", self.catch_turtle)

        self.get_logger().info("Turtle Spawner has been started!")


    def spawn_turtle_callback(self): # callback para chamar o serviço de /spawn
        if not self.spawn_turtle_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /spawn not available")
            return

        spawn_turtle_request_ = Spawn.Request()                    # criar uma requisição para o serviço Spawn
        spawn_turtle_request_.x = random.uniform(2.0, 10.0)        # escolher uma coordenada x aleatória
        spawn_turtle_request_.y = random.uniform(2.0, 10.0)        # escolher uma coordenada y aleatória
        self.get_logger().info(f"Spawning turtle at x: {spawn_turtle_request_.x}, y: {spawn_turtle_request_.y}")


        future = self.spawn_turtle_client_.call_async(spawn_turtle_request_)                            # chamar o serviço de spawn de forma assíncrona
        future.add_done_callback(partial(self.spawn_turtle_response_callback, request=spawn_turtle_request_))   # adicionar um callback para processar a resposta do serviço



    def spawn_turtle_response_callback(self, future, request): # callback para processar a resposta do serviço de /spawn
        try:
            response = future.result()                                   # obter a resposta do serviço
            self.get_logger().info(f"Spawned turtle: {response.name}")   # registrar uma mensagem de log
            
            # criar objeto Turtle para a tartaruga spawnada
            turtle = Turtle()
            turtle.name = response.name
            turtle.x = request.x
            turtle.y = request.y

            self.alive_turtles_.append(turtle)                         # adicionar a tartaruga à lista de vivas
            self.publish_alive_turtles()                               # publicar a lista de tartarugas vivas
        
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle: {e}")



    def publish_alive_turtles(self): # publicar a lista de tartarugas vivas
        msg = TurtleArray()                          # criar uma mensagem do tipo TurtleArray
        msg.turtles = self.alive_turtles_            # definir a lista de tartarugas vivas
        self.alive_turtles_publisher_.publish(msg)   # publicar a mensagem



    # implementar o serviço /catch_turtle que recebe o nome de uma tartaruga e chama o serviço /kill para removê-la da tela do turtlesim
    def catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        turtle_name = request.name
        self.get_logger().info(f"Catching turtle: {turtle_name}")

        if not self.kill_turtle_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /kill not available")
            response.success = False
            response.message = "Service /kill not available"
            return response

        kill_turtle_request = Kill.Request()                 # criar uma requisição para o serviço Kill
        kill_turtle_request.name = turtle_name               # definir o nome da tartaruga a ser removida

        try:
            # chamar o serviço de kill de forma síncrona
            kill_response = self.kill_turtle_client_.call(kill_turtle_request)
            self.get_logger().info(f"Kill service response: {kill_response}")
            
            # atualizar a lista de tartarugas vivas apenas se o kill foi bem-sucedido
            self.alive_turtles_ = [turtle for turtle in self.alive_turtles_ if turtle.name != turtle_name]

            # publicar a lista atualizada
            self.publish_alive_turtles()

            response.success = True
            response.message = f"Successfully caught turtle {turtle_name}"
            
        except Exception as e:
            self.get_logger().error(f"Failed to call kill service: {e}")
            response.success = False
            response.message = f"Failed to kill turtle {turtle_name}: {e}"

        return response

























    # def catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
    #     turtle_name = request.name
    #     self.get_logger().info(f"Catching turtle: {turtle_name}")

    #     # chamar o serviço /kill para remover a tartaruga
    #     if not self.kill_turtle_client_.wait_for_service(timeout_sec=1.0): # esperar até que o serviço esteja disponível
    #         self.get_logger().error("Service /kill not available") 
    #         response.success = False
    #         response.message = "Service /kill not available"
    #         return response
        
    #     kill_turtle_request = Kill.Request()                 # criar uma requisição para o serviço Kill
    #     kill_turtle_request.name = turtle_name               # definir o nome da tartaruga a ser removida

    #     try:
    #         # chamar o serviço de kill de forma síncrona
    #         kill_response = kill_turtle_client.call(kill_turtle_request)
    #         self.get_logger().info(f"Kill service response: {kill_response}")
            
    #         # atualizar a lista de tartarugas vivas apenas se o kill foi bem-sucedido
    #         self.alive_turtles_ = [turtle for turtle in self.alive_turtles_ if turtle.name != turtle_name]

    #         # publicar a lista atualizada
    #         self.publish_alive_turtles()

    #         response.success = True
    #         response.message = f"Successfully caught turtle {turtle_name}"
            
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to call kill service: {e}")
    #         response.success = False
    #         response.message = f"Failed to kill turtle {turtle_name}: {e}"

    #     return response


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleSpawnerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()