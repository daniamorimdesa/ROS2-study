#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64
import random

"""
Este node implementa um sensor fake de temperatura.
- publisher: publica no tópico "temperature"
- Gera temperaturas aleatórias entre 15°C e 45°C
- Limite que "dispara" alarme: 40°C
"""

class SensorNode(Node):

    def __init__(self):
        super().__init__("sensor_node")

        # declarar parâmetros do sensor
        self.declare_parameter("min_temp", 15.0)        # temperatura mínima
        self.declare_parameter("max_temp", 45.0)        # temperatura máxima
        self.declare_parameter("temp_limit", 40.0)      # limite de temperatura para alarme
        self.declare_parameter("timer_period", 1.0)     # período de publicação em segundos
        
        # obter valores dos parâmetros
        self.min_temp_ = self.get_parameter("min_temp").value
        self.max_temp_ = self.get_parameter("max_temp").value
        self.temp_limit_ = self.get_parameter("temp_limit").value
        self.timer_period_ = self.get_parameter("timer_period").value 

        # criar um publicador no tópico "temperature" com fila de tamanho 1
        self.publisher_ = self.create_publisher(Float64, "temperature", 1)  

        # criar um timer que chama publish_temperature a cada timer_period_ segundos
        self.timer_ = self.create_timer(self.timer_period_, self.publish_temperature) 

        self.get_logger().info('\033[92mTemperature Sensor has been started!\033[0m')
        self.get_logger().info(f'Temperature range: {self.min_temp_}°C to {self.max_temp_}°C')
        self.get_logger().warn(f'Alert threshold: {self.temp_limit_}°C')

    # função para publicar a temperatura
    def publish_temperature(self): 

        # Gerar temperatura aleatória no range definido
        temperature = random.uniform(self.min_temp_, self.max_temp_)

        # publicar a mensagem
        msg = Float64()                 
        msg.data = temperature         
        self.publisher_.publish(msg)    
        
def main(args=None):
    rclpy.init(args=args)         
    node = SensorNode()           
    rclpy.spin(node)              
    rclpy.shutdown()              
#--------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()