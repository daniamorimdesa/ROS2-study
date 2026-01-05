#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64

"""Este node cria um sistema de monitoramento que inicia um alarme(print no terminal)
quando a temperatura medida ultrapassa um limite predefinido.
- subscriber: escuta o tópico "temperature"
limite pré-definido de temperatura: 40.0 graus Celsius
""" 
class MonitorNode(Node):

    def __init__(self):
        super().__init__("monitor_node") 

        # declarar parâmetro do limite de temperatura
        self.declare_parameter("temp_limit", 40.0)

        # obter valor do parâmetro
        self.temp_limit_ = self.get_parameter("temp_limit").value

        # criar um subscriber no tópico "temperature" com fila de tamanho 1
        self.subscriber_ = self.create_subscription(Float64, "temperature", self.callback_temperature, 1) 

        self.get_logger().info('\033[92mTemperature Monitor has been started!\033[0m') 

    # função de callback que processa a mensagem recebida
    def callback_temperature(self, msg: Float64): 

        # verificar se a temperatura recebida ultrapassa o limite e "iniciar alarme"(print no terminal)
        if msg.data > self.temp_limit_:
            self.get_logger().warn(f"ALARM! High Temperature Detected: {msg.data:.2f}°C")

        # exibir o valor lido pelo sensor na faixa normal
        else:
            self.get_logger().info(f"Temperature: {msg.data:.2f} °C") 
    
def main(args=None):
    rclpy.init(args=args)      
    node = MonitorNode() 
    rclpy.spin(node)          
    rclpy.shutdown()           
#-------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()