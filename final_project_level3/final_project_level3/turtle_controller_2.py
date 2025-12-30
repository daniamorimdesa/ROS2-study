#!/usr/bin/env python3
from functools import partial
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import Kill, Spawn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from my_robot_interfaces.action import MoveTurtle
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # criar um nó chamado "turtle_controller"

        # criar um callback group reentrante para permitir a execução concorrente dos callbacks
        self.cb_group_ = ReentrantCallbackGroup()
#-----------------------------------------------------------------------------------------------------------------------------------------------
        # declarar parâmetros
        self.declare_parameter("turtle_name", "turtle1") # valor padrão "turtle1"

        # obter os valores dos parâmetros
        self.turtle_name_ = self.get_parameter("turtle_name").value  
#-----------------------------------------------------------------------------------------------------------------------------------------------
        # criar dois service clients para os services "/kill" e "/spawn"
        self.kill_client_ = self.create_client(Kill, "/kill", callback_group=self.cb_group_)     
        self.spawn_client_ = self.create_client(Spawn, "/spawn", callback_group=self.cb_group_)
#-----------------------------------------------------------------------------------------------------------------------------------------------
        # criar um timer para disparar a sequência de spawn e kill
        # Step 2: spawn a turtle com o nome do parâmetro (ex: turtle2)
        self.start_timer = self.create_timer(1.0, self.start_sequence)
#-----------------------------------------------------------------------------------------------------------------------------------------------
        # criar publisher de velocidade para publicar durante execução da action
        self.cmd_vel_pub_ = self.create_publisher(Twist, f"/{self.turtle_name_}/cmd_vel", 10)
#-----------------------------------------------------------------------------------------------------------------------------------------------
        # criar o Action Server e variaveis de controle da action
        self.goal_lock_ = threading.Lock()          # criar um bloqueio para gerenciar o acesso à meta atual
        self.goal_handle_ : ServerGoalHandle = None # armazenar o manipulador da meta atual
        self.move_turtle_server_ = ActionServer(
            self,
            MoveTurtle,
            "move_turtle_" + self.turtle_name_,
            goal_callback = self.goal_callback,
            handle_accepted_callback = self.handle_accepted_callback,
            cancel_callback = self.cancel_callback,
            execute_callback = self.execute_callback,
            callback_group=self.cb_group_,
        )
#-----------------------------------------------------------------------------------------------------------------------------------------------
        self.get_logger().info("Turtle Controller has been started!")
#-----------------------------------------------------------------------------------------------------------------------------------------------
    # ao iniciar o nó, ele automaticamente começa a sequência
    def start_sequence(self):

        # cancelar o timer (ele roda apenas uma vez)
        self.start_timer.cancel()

        # chamar spawn da tartaruga
        self.spawn_turtle()
#-----------------------------------------------------------------------------------------------------------------------------------------------
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
#-----------------------------------------------------------------------------------------------------------------------------------------------
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

            # Step 2: NÃO matar a tartaruga - ela fica viva para ser controlada pela action
            # # após spawnar, esperar 3 segundos e matar a tartaruga
            # # se self.kill_timer já foi criado e ainda está ativo, cancela antes de criar outro
            # if hasattr(self, 'kill_timer') and self.kill_timer.is_alive():
            #     self.kill_timer.cancel()
            # self.kill_timer = self.create_timer(3.0, self.start_kill)
#-----------------------------------------------------------------------------------------------------------------------------------------------
    # iniciar o timer para matar a tartaruga
    def start_kill(self):

        # cancelar o timer (ele roda apenas uma vez)
        self.kill_timer.cancel()

        # chamar o serviço de matar a tartaruga
        self.kill_turtle()
#-----------------------------------------------------------------------------------------------------------------------------------------------
    # Callbacks do Action Server
#-----------------------------------------------------------------------------------------------------------------------------------------------
    # função de callback para processar a meta
    def goal_callback(self, goal_request: MoveTurtle.Goal):

        self.get_logger().info("Received a new goal")

        # Policy: rejeitar se já tem goal ativa
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("A goal is already active, rejecting new goal...")
                return GoalResponse.REJECT   

        # Validar goal request
        # rejeitar se abs(linear_vel_x) > 3.0 ou abs(angular_vel_z) > 2.0 ou duration_sec <= 0
        if abs(goal_request.linear_vel_x) > 3.0 or abs(goal_request.angular_vel_z) > 2.0 or goal_request.duration_sec <= 0:
            self.get_logger().warn("Rejecting the goal...") 
            return GoalResponse.REJECT                      
        
        self.get_logger().info("Accepting the goal!") # log de aceitação da meta
        return GoalResponse.ACCEPT                    # aceitar a meta


    # função de callback chamada quando uma meta é aceita
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        # Reservar a ação(antes de execute) para prevenir race condition
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        # Iniciar a execução
        goal_handle.execute()


    # função de callback para cancelar a meta (sempre aceitar cancelamento)
    def cancel_callback(self, goal_handle: ServerGoalHandle): 
        self.get_logger().info("Received a cancel request!") 
        return CancelResponse.ACCEPT      
                    

    # função de callback para executar a ação
    def execute_callback(self, goal_handle: ServerGoalHandle): 

        # goal_handle_ já foi setado no handle_accepted_callback

        # ler linear_vel_x, angular_vel_z, duration_sec
        linear_vel_x = goal_handle.request.linear_vel_x
        angular_vel_z = goal_handle.request.angular_vel_z
        duration_sec = goal_handle.request.duration_sec

        self.get_logger().info("Executing the goal...")
        
        # start_time = now()
        start_time = self.get_clock().now()
        
        # criar mensagem Twist com as velocidades do goal
        twist = Twist()
        twist.linear.x = linear_vel_x
        twist.angular.z = angular_vel_z
        
        # enquanto now() - start_time < duration_sec:
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:
            # se o goal foi cancelado
            if goal_handle.is_cancel_requested:
                # publicar Twist(0,0)
                stop_twist = Twist()
                self.cmd_vel_pub_.publish(stop_twist)
                
                # marcar como cancelado
                goal_handle.canceled()
                
                # limpar self.goal_handle_ (pra permitir o próximo goal)
                with self.goal_lock_:
                    self.goal_handle_ = None
                
                # retornar Result(success=False, message="Canceled")
                self.get_logger().info("Goal canceled!")
                result = MoveTurtle.Result()
                result.success = False
                result.message = "Canceled"
                return result
            
            # publicar Twist com as velocidades do goal
            self.cmd_vel_pub_.publish(twist)
            
            # dormir para manter 10 Hz (0.1 s)
            time.sleep(0.1)
        
        # quando acabar o tempo:
        # publicar Twist(0,0)
        stop_twist = Twist()
        self.cmd_vel_pub_.publish(stop_twist)
        
        # marcar succeed
        goal_handle.succeed()
        
        # limpar self.goal_handle_ (pra permitir o próximo goal)
        with self.goal_lock_:
            self.goal_handle_ = None
        
        # retornar Result(success=True, message="Success")
        self.get_logger().info("Goal succeeded! :)")
        result = MoveTurtle.Result()
        result.success = True
        result.message = "Success"
        return result


#-----------------------------------------------------------------------------------------------------------------------------------------------
# Função principal
#-----------------------------------------------------------------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)         # inicializar o rclpy
    node = TurtleControllerNode() # instanciar o nó
    
    # criar um executor multithreaded para permitir execução concorrente dos callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()              # finalizar o rclpy
#-----------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
    