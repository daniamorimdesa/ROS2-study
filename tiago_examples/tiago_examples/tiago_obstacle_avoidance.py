#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

"""
Nó que faz o robô Tiago andar pelo mundo evitando obstáculos.
Lê o LaserScan e ajusta velocidades linear e angular para não colidir.
"""

class TiagoObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("tiago_obstacle_avoidance")
        
        # Parâmetros de segurança
        self.safe_distance_ = 2.0    # distância mínima de segurança (m)
        self.danger_distance_ = 1.5  # distância de perigo (m)
        
        # Velocidades
        self.max_linear_vel_ = 0.5
        self.max_angular_vel_ = 1.0
        
        # Subscriber do laser
        self.laser_sub_ = self.create_subscription(LaserScan, "/scan_raw", self.laser_callback, 10)
        
        # Publisher de velocidade
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.get_logger().info("Tiago Obstacle Avoidance started!")
    
    def laser_callback(self, msg: LaserScan):
        """Processa laser scan e publica velocidade para evitar obstáculos"""
        
        # Dividir laser em regiões mais amplas
        num_readings = len(msg.ranges)
        
        # Região frontal AMPLIADA: 50% central (mais sensível)
        # Esquerda: 25% esquerda
        # Direita: 25% direita
        
        front_start = num_readings // 4
        front_end = 3 * num_readings // 4
        
        left_start = 3 * num_readings // 4
        left_end = num_readings
        
        right_start = 0
        right_end = num_readings // 4
        
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
        
        # Considerar também a distância mínima geral (mais conservador)
        overall_ranges = [r if r > msg.range_min and r < msg.range_max else msg.range_max
                         for r in msg.ranges if not np.isnan(r)]
        overall_min = min(overall_ranges) if overall_ranges else msg.range_max
        
        # Criar mensagem Twist
        twist = Twist()
        
        # Usar a distância mais conservadora (menor entre front e overall)
        critical_distance = min(front_min, overall_min)
        
        # Lógica de navegação
        if critical_distance > self.safe_distance_:
            # Caminho livre à frente - seguir em frente
            twist.linear.x = self.max_linear_vel_
            twist.angular.z = 0.0
            self.get_logger().info(f"Seguindo em frente | front={front_min:.2f}m | min={critical_distance:.2f}m", 
                                  throttle_duration_sec=1.0)
        
        elif critical_distance > self.danger_distance_:
            # Obstáculo próximo - desacelerar e começar a virar
            twist.linear.x = self.max_linear_vel_ * 0.3
            
            # Virar para o lado mais livre
            if left_min > right_min:
                twist.angular.z = self.max_angular_vel_ * 0.8  # virar à esquerda
                self.get_logger().info(f"Obstáculo próximo, virando ESQUERDA | front={front_min:.2f}m | L={left_min:.2f} R={right_min:.2f}", 
                                      throttle_duration_sec=0.5)
            else:
                twist.angular.z = -self.max_angular_vel_ * 0.8  # virar à direita
                self.get_logger().info(f"Obstáculo próximo, virando DIREITA | front={front_min:.2f}m | L={left_min:.2f} R={right_min:.2f}", 
                                      throttle_duration_sec=0.5)
        
        else:
            # Obstáculo muito próximo - parar e virar rapidamente
            twist.linear.x = 0.0
            
            # Virar para o lado mais livre
            if left_min > right_min:
                twist.angular.z = self.max_angular_vel_  # virar à esquerda
                self.get_logger().warn(f"PERIGO! Virando ESQUERDA | front={front_min:.2f}m | min={critical_distance:.2f}m", 
                                      throttle_duration_sec=0.3)
            else:
                twist.angular.z = -self.max_angular_vel_  # virar à direita
                self.get_logger().warn(f"PERIGO! Virando DIREITA | front={front_min:.2f}m | min={critical_distance:.2f}m", 
                                      throttle_duration_sec=0.3)
        
        # Publicar velocidade
        self.cmd_vel_pub_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
