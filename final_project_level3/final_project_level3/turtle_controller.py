#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import Kill, Spawn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

"""
Step 1 do final project level 3:
Este node controla o spawn e a remoção de uma tartaruga no turtlesim.
Ele usa dois service clients para chamar os serviços /spawn e /kill do turtlesim.
O nome da tartaruga a ser controlada pode ser configurado via parâmetro.

"""

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # criar um nó chamado "turtle_controller"

        # criar um callback group reentrante para permitir a execução concorrente dos callbacks
        self.cb_group_ = ReentrantCallbackGroup()

        # declarar parâmetros
        self.declare_parameter("turtle_name", "turtle1") # valor padrão "turtle1"

        # obter os valores dos parâmetros
        self.turtle_name_ = self.get_parameter("turtle_name").value  

        # criar dois service clients para os services "/kill" e "/spawn"
        self.kill_client_ = self.create_client(Kill, "/kill", callback_group=self.cb_group_)     
        self.spawn_client_ = self.create_client(Spawn, "/spawn", callback_group=self.cb_group_)

        # criar um timer para disparar a sequência de spawn e kill
        self.start_timer = self.create_timer(1.0, self.start_sequence)

        self.get_logger().info("Turtle Controller has been started!")


    # ao iniciar o nó, ele automaticamente começa a sequência
    def start_sequence(self):

        # cancelar o timer (ele roda apenas uma vez)
        self.start_timer.cancel()

        # chamar spawn da tartaruga
        self.spawn_turtle()


    # chamar o serviço de matar a tartaruga
    def kill_turtle(self):
        # esperar até que o serviço esteja disponível
        while not self.kill_client_.wait_for_service(1.0): 
            self.get_logger().warn("Waiting for kill service...")
        
        request = Kill.Request()                       # criar uma requisição para o serviço Kill
        request.name = self.turtle_name_               # definir o nome da tartaruga a ser morta

        self.get_logger().info(f"Trying to remove turtle...")

        # chamar o serviço de kill de forma assíncrona
        future = self.kill_client_.call_async(request)

        # callback para processar a resposta do serviço de kill
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name=self.turtle_name_))

    # callback para processar a resposta do serviço de kill
    def callback_call_kill_service(self, future, turtle_name):

        # obter a resposta do serviço
        try:
            response: Kill.Response = future.result()      
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        self.get_logger().info(f"Turtle {turtle_name} has been removed.")


    # chamar o serviço de spawnar a tartaruga
    def spawn_turtle(self):

        # esperar até que o serviço esteja disponível
        while not self.spawn_client_.wait_for_service(1.0): 
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()         # criar uma requisição para o serviço Spawn
        request.x = 5.0                   # definir a coordenada x
        request.y = 5.0                   # definir a coordenada y
        request.theta = 0.0               # definir a orientação theta
        request.name = self.turtle_name_  # definir o nome da tartaruga

        self.get_logger().info(f"Trying to spawn turtle...")

        # chamar o serviço de spawn de forma assíncrona
        future = self.spawn_client_.call_async(request) 
                                           
        # adicionar um callback para processar a resposta do serviço                                    
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request)) 

    # callback para processar a resposta do serviço de spawn
    def callback_call_spawn_service(self, future, request: Spawn.Request):

        # obter a resposta do serviço
        try:
            response: Spawn.Response = future.result()      
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        # se a tartaruga foi spawnada com sucesso
        if response.name != "":                                           
            self.get_logger().info(f"New alive turtle: {response.name}")

            # após spawnar, esperar 3 segundos e matar a tartaruga
            # se self.kill_timer já foi criado e ainda está ativo, cancela antes de criar outro
            if hasattr(self, 'kill_timer') and self.kill_timer.is_alive():
                self.kill_timer.cancel()
            self.kill_timer = self.create_timer(3.0, self.start_kill)


    # iniciar o timer para matar a tartaruga
    def start_kill(self):

        # cancelar o timer (ele roda apenas uma vez)
        self.kill_timer.cancel()

        # chamar o serviço de matar a tartaruga
        self.kill_turtle()



def main(args=None):
    rclpy.init(args=args)         # inicializar o rclpy
    node = TurtleControllerNode() # instanciar o nó
    
    # criar um executor multithreaded para permitir execução concorrente dos callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()              # finalizar o rclpy
#----------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()