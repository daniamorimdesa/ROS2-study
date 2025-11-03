#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LEDPanelStatus
from my_robot_interfaces.srv import SetLED

# Step 1: create LED panel node and publish the LED panel state (with a custom message)
# Step 2: add a service server inside the LED panel node (with a custom service definition)

class LEDPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel") # criar um nó chamado "led_panel"

        # declarar parâmetros
        # parâmetros para os estados dos LEDs
        self.declare_parameter("led_status", [0,0,0]) # declarar o parâmetro "led_status" com valor padrão [0,0,0]
        self.led_status = self.get_parameter("led_status").value # obter o valor do parâmetro "led_status"

        self.led_status_publisher_ = self.create_publisher(LEDPanelStatus, "led_panel_status", 10) # criar um publicador no tópico "led_panel_status" com fila de tamanho 10
        self.timer_ = self.create_timer(1.0, self.publish_led_panel_status)    # criar um timer que chama publish_led_panel_status a cada 1 segundo
        # self.led_status = [0, 0, 0]  # estado inicial dos LEDs

        # criar um servidor de serviço chamado "set_led" que usa o tipo SetLED e chama callback_set_led quando uma requisição é recebida
        self.server_ = self.create_service(SetLED, "set_led", self.callback_set_led)
        self.get_logger().info("LED Panel has been started!") # registrar uma mensagem de log

    def publish_led_panel_status(self):
        msg = LEDPanelStatus() # criar uma mensagem do tipo LEDPanelStatus (integer array)
        msg.led_status =  self.led_status # definir o status dos LEDs
        self.led_status_publisher_.publish(msg) # publicar a mensagem
        self.get_logger().info(f"LED status: {self.led_status}") # registrar o status atual dos LEDs

    def callback_set_led(self, request: SetLED.Request, response: SetLED.Response):
        # checar se a request é válida(state:on ou off, número 0, 1 ou 2)
        led_number = request.led_number
        state = request.led_state

        if state not in ["on","off"] or led_number not in [0, 1, 2]:
            response.success = False
            self.get_logger().info(f"Invalid LED request: {request.led_number}, {request.led_state}")
            return response
        
        self.led_status[led_number] = 1 if state == "on" else 0
        response.success = True
        self.get_logger().info(f"Set LED {led_number+1} to {state}. Current status: {self.led_status}")
        return response


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = LEDPanelNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()