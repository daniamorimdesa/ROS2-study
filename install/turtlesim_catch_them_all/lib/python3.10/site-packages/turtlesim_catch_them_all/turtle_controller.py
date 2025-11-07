#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, pi
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

"""
- Rodar um loop de controle(usando um timer de periodo curto) para ir até o ponto alvo.
- A primeira tartaruga na tela(turtle1) é a mestre que vai ser controlada, para isso é necessário dar subscribe
no tópico /turtle1/pose para saber a posição atual dela e publicar na /turtle1/cmd_vel para mover ela.
- O loop de controle vai usar um controlador proporcional simples para calcular a velocidade linear e angular.
- Dar subscribe pro tópico /alive_turtles para saber a posição das outras tartarugas na tela, dessa informação,
selecionar a tartaruga mais próxima da turtle1 como alvo.
- Quando uma tartaruga for pega, chamar o serviço /catch_turtle avisado pelo nó turtle_spawner

interfaces necessárias:
- Turtle.msg e TurtleArray.msg para mandar a lista de tartarugas (nome e coordenadas) no tópico /alive_turtles
- CatchTurtle.srv para avisar o nome da tartaruga que foi pega, o cliente vai ser o turtle_controller e o servidor o turtle_spawner

Controlador proporcional simples: calcula a ação de controle(a velocidade linear ou angular) proporcionalmente ao erro
u(t) = Kp * e(t)
onde: 
u(t)= saída de controle (velocidade linear ou angular)
Kp = ganho proporcional (ajusta a intensidade da resposta do controlador)
e(t) = erro (diferença entre a posição atual e a posição alvo)

velocidade linear-> v = Kp_linear * erro_linear ; onde erro_linear = distância até o alvo
velocidade angular-> w = Kp_angular * erro_angular


"""

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # criar um nó chamado "turtle_controller"

        # subscribe pro /turtle1/pose para saber a posição atual da turtle1
        self.turtle1_pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_turtle1_pose, 10)

        # publicar na /turtle1/cmd_vel para mover a turtle1
        self.turtle1_cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # criar um timer para o loop de controle
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop_callback)
        self.position_ = Pose()  # armazenar a posição atual da turtle1
        self.target_position_ = None  # armazenar a posição alvo (de outra tartaruga)

        # subscribe pro tópico /alive_turtles para saber a posição das outras tartarugas na tela
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "/alive_turtles", self.callback_alive_turtles, 10)

        # Quando uma tartaruga for pega, chamar o serviço /catch_turtle avisado pelo nó turtle_spawner
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "/catch_turtle")
        self.catch_turtle_request_ = CatchTurtle.Request()

        self.get_logger().info("Turtle Controller has been started!") # registrar uma mensagem de log


    def _normalize_angle(self, a): # normalizar ângulo entre -pi e pi
        if a >  pi: a -= 2*pi
        if a < -pi: a += 2*pi
        return a
    
    def _stop(self): # parar a turtle1
        msg = Twist()
        self.turtle1_cmd_vel_publisher_.publish(msg)


    def control_loop_callback(self): # callback do loop de controle
        if self.target_position_ is None:
            self._stop()
            return  # se não houver posição alvo, sair

        # calcular a diferença entre a posição atual e a posição alvo
        diff_x = self.target_position_.x - self.position_.x
        diff_y = self.target_position_.y - self.position_.y

        # calcular a distância e o ângulo para o alvo
        distance = (diff_x**2 + diff_y**2)**0.5
        angle_to_target = atan2(diff_y, diff_x)

        # calcular o erro angular  (onde quero estar - onde estou)
        # se for positivo, preciso girar no sentido anti-horário;
        # se for negativo, girar no sentido horário
        error_theta = self._normalize_angle(angle_to_target - self.position_.theta)


        # controlador proporcional simples
        K_linear = 2.0 
        K_angular = 6.0

        # ganho proporcional + saturação
        linear_velocity  = max(min(K_linear  * distance, 2.0), 0.0)  # 0..2 m/s
        angular_velocity = max(min(K_angular * error_theta, 4.0), -4.0)  # -4..4 rad/s

        cmd_vel_msg = Twist()
        # heurística: se desalinhado, prioriza girar
        if abs(error_theta) > 0.3:
            cmd_vel_msg.linear.x = 0.0
        else:
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = angular_velocity

        self.turtle1_cmd_vel_publisher_.publish(cmd_vel_msg) # publicar a mensagem de velocidade para mover a turtle1


        # linear_velocity = K_linear * distance
        # angular_velocity = K_angular * error_theta

        # # criar a mensagem de velocidade
        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = linear_velocity
        # cmd_vel_msg.angular.z = angular_velocity

        # # publicar a mensagem de velocidade para mover a turtle1
        # self.turtle1_cmd_vel_publisher_.publish(cmd_vel_msg)




        # verificar se chegou perto da tartaruga alvo (capturou)
        if distance < 0.5:
            self._stop()
            self.call_catch_turtle_service(self.target_position_.name)  # chamar serviço /catch_turtle
            self.target_position_ = None                                # limpar o alvo após capturar



            # self.call_catch_turtle_service(self.target_position_.name)
            # self.target_position_ = None  # limpar o alvo após capturar


    def callback_turtle1_pose(self, msg: Pose): # callback para a posição da turtle1
        self.position_ = msg  # atualizar a posição atual da turtle1

    def callback_alive_turtles(self, msg: TurtleArray):
        if not msg.turtles:
            self.target_position_ = None
            return

        # se já tenho alvo e ele ainda está na lista, mantenho
        if self.target_position_:
            names = [t.name for t in msg.turtles]
            if self.target_position_.name in names:
                return  # mantém alvo atual

        # escolhe o mais próximo
        closest = min(msg.turtles,key=lambda t: ((t.x - self.position_.x)**2 + (t.y - self.position_.y)**2)**0.5)
        self.target_position_ = closest




    # def callback_alive_turtles(self, msg: TurtleArray): # callback para a lista de tartarugas vivas
    #     if not msg.turtles:
    #         self.target_position_ = None  # se não houver tartarugas vivas, não há alvo
    #         return

    #     # encontrar a tartaruga mais próxima
    #     closest_turtle = min(msg.turtles,key=lambda t: ((t.x - self.position_.x)**2 + (t.y - self.position_.y)**2)**0.5)
        
    #     # armazenar a tartaruga alvo (incluindo o nome para captura)
    #     self.target_position_ = closest_turtle  # atualizar a posição alvo para a tartaruga mais próxima

    def call_catch_turtle_service(self, turtle_name): # chamar o serviço /catch_turtle
        if not self.catch_turtle_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /catch_turtle not available")
            return
        
        request = CatchTurtle.Request()
        request.name = turtle_name
        
        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(self.catch_turtle_response_callback)
    
    def catch_turtle_response_callback(self, future): # callback para a resposta do serviço /catch_turtle
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully caught turtle: {response.message}")
            else:
                self.get_logger().error(f"Failed to catch turtle: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")



def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = TurtleControllerNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()