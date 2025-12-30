import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry # importar pose do TIAgo
from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi

"""
Adaptar código do turtlesim(turtle_pose_robotica.py) para o robô TIAgo, 
que deve se mover para uma posição desejada, publicando no tópico /cmd_vel. 
O robô deve rotacionar até alinhar com a direção do destino e depois se mover 
em linha reta até chegar na posição desejada. 

Dica: se quiser “odom realista”, troque /ground_truth_odom por /mobile_base_controller/odom.
"""

class TiagoPoseNode(Node):
    def __init__(self):
        super().__init__("tiago_pose_node") # criar um nó chamado "tiago_pose_node"

        # armazenar a pose atual do robô
        self.x_ = None
        self.y_ = None
        self.yaw_ = None
 
        # flags de controle
        self.rotationIsOver = False # flag para indicar se a rotação foi concluída
        self.goalReached = False    # flag para indicar se chegou no destino

        # posição final desejada (goal)
        self.x_goal = 0.0
        self.y_goal = 0.0

        # ganhos e tolerâncias
        self.k_angular = 4.0  # ganho angular
        self.k_linear = 0.6   # ganho linear
        self.yaw_tol = 0.1  # tolerância angular (cerca de 5.7 graus)
        self.dist_tol = 0.15  # tolerância de distância (aumentada)
        self.max_linear_vel = 0.5  # velocidade linear máxima

        # criar um subscriber para saber a posição atual do TIAgo
        self.odom_subscriber_ = self.create_subscription(Odometry, "/ground_truth_odom", self.callback_odom, 10)

        # criar um publisher na /tiago/cmd_vel para mover o TIAgo
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # criar um timer para controlar o loop de movimento
        self.timer_ = self.create_timer(0.1, self.control_loop) # 10 Hz

        self.get_logger().info("Tiago Pose Node has been started!") # registrar uma mensagem de log

    # função de callback para mensagens de odometria
    def callback_odom(self, msg: Odometry): 

        self.x_ = msg.pose.pose.position.x # armazenar a posição x
        self.y_ = msg.pose.pose.position.y # armazenar a posição y

        q = msg.pose.pose.orientation # extrair a orientação em quaternion

        # converter quaternion para yaw e armazenar
        self.yaw_ = self.quaternion_to_yaw(q.x, q.y, q.z, q.w) 

    # função de controle para mover o TIAgo
    def control_loop(self):
        # garantir que a pose foi recebida
        if self.x_ is None or self.y_ is None or self.yaw_ is None:
            return
        
        # verificar se chegou no destino
        if self.goalReached:
            return
    
        # calcular distâncias
        dist_x = self.x_goal - self.x_         # distância em x até o objetivo
        dist_y = self.y_goal - self.y_         # distância em y até o objetivo
        distance = sqrt(dist_x**2 + dist_y**2) # distância euclidiana até o objetivo

        # calcular ângulo desejado
        goal_yaw = atan2(dist_y, dist_x) 
        diff_angular = goal_yaw - self.yaw_
        
        # normalizar ângulo para [-pi, pi]
        if diff_angular > pi:
            diff_angular -= 2 * pi
        elif diff_angular < -pi:
            diff_angular += 2 * pi

        cmd = Twist() # criar uma nova mensagem Twist

        # Rotacionar até alinhar
        if not self.rotationIsOver:
            cmd.linear.x = 0.0                              # não mover linearmente
            cmd.angular.z = self.k_angular * diff_angular   # velocidade angular proporcional à diferença angular
            self.get_logger().info(f"Rotating: diff_angular={diff_angular:.3f} rad, tolerance={self.yaw_tol}")
             
            if abs(diff_angular) < self.yaw_tol: # se a diferença angular for pequena o suficiente
                self.rotationIsOver = True
                cmd.angular.z = 0.0
                self.get_logger().info("Rotation complete!")
        
        # Mover em linha reta
        if self.rotationIsOver and distance > self.dist_tol:
            # saturar velocidade linear
            linear_vel = self.k_linear * distance
            cmd.linear.x = min(linear_vel, self.max_linear_vel)   # limitar velocidade máxima
            cmd.angular.z = 0.0                       # manter orientação
            self.get_logger().info(f"Moving forward: distance={distance:.2f}, linear.x={cmd.linear.x:.2f}")

        # quando chegar no destino
        if distance <= self.dist_tol and self.rotationIsOver:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.goalReached = True
            self.get_logger().info(f"Goal reached! x={self.x_:.2f}, y={self.y_:.2f}, yaw={self.yaw_:.2f} rad")
        
        self.cmd_vel_publisher_.publish(cmd) # publicar o comando de velocidade
    
    # converter quaternion para yaw (ângulo em torno do eixo z)
    def quaternion_to_yaw(self, x, y, z, w)-> float:

        # yaw (Z) de quaternion
        # yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)  # inicializar o rclpy
    node = TiagoPoseNode() # instanciar o nó
    rclpy.spin(node)       # manter o nó ativo
    rclpy.shutdown()       # finalizar o rclpy
#-----------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()

