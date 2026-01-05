#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

"""
Nó que faz o robô Tiago andar pelo mundo evitando obstáculos.
Lê o LaserScan e ajusta velocidades linear e angular para não colidir.
versão 2: adiciona verificação de segurança 360° para evitar colisões ao girar.
"""

class TiagoObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("tiago_obstacle_avoidance")
        
        # Parâmetros de segurança
        self.safe_distance_ = 0.8      # distância mínima de segurança (m)
        self.danger_distance_ = 0.5    # distância de perigo (m)
        self.emergency_distance_ = 0.3 # distância crítica 360° (m) - bloqueia rotação
        self.side_diff_threshold_ = 0.3 # diferença mínima entre lados para decidir direção (m)
        
        # Velocidades
        self.max_linear_vel_ = 0.5
        self.max_angular_vel_ = 1.0
        
        # Detecção de travamento
        self.stuck_counter_ = 0          # contador de iterações travado
        self.stuck_threshold_ = 20       # 2 segundos a 10Hz
        self.last_rotation_direction_ = 1 # 1=esquerda, -1=direita
        
        # Subscriber do laser
        self.laser_sub_ = self.create_subscription(LaserScan, "/scan_raw", self.laser_callback, 10)
        
        # Publisher de velocidade
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.get_logger().info('\033[92mTiago Obstacle Avoidance started!\033[0m') # log em verde
    
    def laser_callback(self, msg: LaserScan):
        """Processa laser scan e publica velocidade para evitar obstáculos"""
        
        # Dividir laser em regiões mais amplas
        num_readings = len(msg.ranges)
        
        # Região frontal AMPLIADA: 50% central (mais sensível)
        # Esquerda: 25% esquerda
        # Direita: 25% direita

        right_start = 0                       # início da região direita
        right_end = num_readings // 4         # fim da região direita
        
        front_start = num_readings // 4       # início da região frontal
        front_end = 3 * num_readings // 4     # fim da região frontal
        
        left_start = 3 * num_readings // 4    # início da região esquerda
        left_end = num_readings               # fim da região esquerda
        

        
        # Extrair ranges válidos (ignorar inf, -inf, nan e valores fora do range)
        # Note: -inf significa "sem medição", tratar como range_max
        front_ranges = [r if r > msg.range_min and r < msg.range_max else msg.range_max 
                       for r in msg.ranges[front_start:front_end] 
                       if not np.isnan(r)]
        left_ranges = [r if r > msg.range_min and r < msg.range_max else msg.range_max
                      for r in msg.ranges[left_start:left_end] 
                      if not np.isnan(r)]
        right_ranges = [r if r > msg.range_min and r < msg.range_max else msg.range_max
                       for r in msg.ranges[right_start:right_end] 
                       if not np.isnan(r)]
        
        # Calcular distâncias mínimas em cada região
        front_min = min(front_ranges) if front_ranges else msg.range_max
        left_min = min(left_ranges) if left_ranges else msg.range_max
        right_min = min(right_ranges) if right_ranges else msg.range_max
        
        # Calcular distância mínima geral (360°) para verificação de segurança em rotações
        overall_ranges = [r if r > msg.range_min and r < msg.range_max else msg.range_max
                         for r in msg.ranges if not np.isnan(r)]
        overall_min = min(overall_ranges) if overall_ranges else msg.range_max
        
        # Criar mensagem Twist
        twist = Twist()
        
        # Lógica de navegação:
        # - Movimento para frente: só considera o que está na frente
        # - Rotação: verifica 360° para evitar bater ao girar
        
        # Lógica de navegação (baseada apenas em front_min)
        if front_min > self.safe_distance_:
            # Caminho livre à frente - seguir em frente
            twist.linear.x = self.max_linear_vel_
            twist.angular.z = 0.0
            self.get_logger().info(f"Seguindo em frente | front={front_min:.2f}m", 
                                  throttle_duration_sec=1.0)
        
        elif front_min > self.danger_distance_:
            # Obstáculo próximo - desacelerar e começar a virar
            twist.linear.x = self.max_linear_vel_ * 0.3
            
            # Virar para o lado mais livre COM HYSTERESIS (evita oscilação)
            if left_min > right_min + self.side_diff_threshold_:
                twist.angular.z = self.max_angular_vel_ * 0.8  # virar à esquerda
                self.last_rotation_direction_ = 1
                self.get_logger().info(f"Obstáculo próximo, virando ESQUERDA | front={front_min:.2f}m | L={left_min:.2f} R={right_min:.2f}", 
                                      throttle_duration_sec=0.5)
            elif right_min > left_min + self.side_diff_threshold_:
                twist.angular.z = -self.max_angular_vel_ * 0.8  # virar à direita
                self.last_rotation_direction_ = -1
                self.get_logger().info(f"Obstáculo próximo, virando DIREITA | front={front_min:.2f}m | L={left_min:.2f} R={right_min:.2f}", 
                                      throttle_duration_sec=0.5)
            else:
                # Lados similares: mantém direção anterior
                twist.angular.z = self.last_rotation_direction_ * self.max_angular_vel_ * 0.8
                self.get_logger().info(f"Lados similares, mantendo direção | front={front_min:.2f}m | L={left_min:.2f} R={right_min:.2f}", 
                                      throttle_duration_sec=0.5)
        
        else:
            # Obstáculo muito próximo na frente - parar movimento linear e virar
            twist.linear.x = 0.0
            
            # Virar para o lado mais livre COM HYSTERESIS
            if left_min > right_min + self.side_diff_threshold_:
                twist.angular.z = self.max_angular_vel_  # virar à esquerda
                self.last_rotation_direction_ = 1
                self.get_logger().warn(f"PERIGO! Virando ESQUERDA | front={front_min:.2f}m", 
                                      throttle_duration_sec=0.3)
            elif right_min > left_min + self.side_diff_threshold_:
                twist.angular.z = -self.max_angular_vel_  # virar à direita
                self.last_rotation_direction_ = -1
                self.get_logger().warn(f"PERIGO! Virando DIREITA | front={front_min:.2f}m", 
                                      throttle_duration_sec=0.3)
            else:
                # Lados similares: mantém direção anterior
                twist.angular.z = self.last_rotation_direction_ * self.max_angular_vel_
                self.get_logger().warn(f"PERIGO! Lados similares, mantendo direção | front={front_min:.2f}m", 
                                      throttle_duration_sec=0.3)
        
        # VERIFICAÇÃO DE SEGURANÇA 360°: bloquear rotação se muito próximo de algo
        if overall_min < self.emergency_distance_:
            twist.angular.z = 0.0  # cancela rotação por segurança
            twist.linear.x = 0.0   # para completamente
            self.get_logger().error(f"EMERGÊNCIA 360°! Obstáculo a {overall_min:.2f}m - PARADO!", 
                                   throttle_duration_sec=0.2)
        
        # DETECÇÃO DE TRAVAMENTO: se rotacionando sem andar, anda para trás
        if twist.linear.x < 0.1 and abs(twist.angular.z) > 0.5:
            self.stuck_counter_ += 1
        else:
            self.stuck_counter_ = 0
        
        # Se travado por mais de 2 segundos, executar manobra de escape
        if self.stuck_counter_ > self.stuck_threshold_:
            self.get_logger().warn(f"TRAVADO! Executando manobra de escape...", 
                                  throttle_duration_sec=0.5)
            twist.linear.x = -0.3  # anda para trás
            twist.angular.z = self.last_rotation_direction_ * self.max_angular_vel_  # vira ao mesmo tempo
            self.stuck_counter_ = 0  # reseta contador após iniciar escape
        
        # Publicar velocidade
        self.cmd_vel_pub_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
