#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

"""
-> Versão do curso

Este node cria um nó de bateria que monitora o estado da bateria (cheia ou vazia)
e controla um LED para indicar o estado da bateria.
- Se a bateria estiver cheia, o LED 3 estará desligado.
- Se a bateria estiver vazia, o LED 3 estará ligado.
- O estado da bateria alterna entre cheio e vazio com base em intervalos de tempo.

- service client: chama o serviço "set_led" para alterar o estado do LED
"""

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery") # criar um nó chamado "battery"

        self.battery_state_ = "full" # estado inicial da bateria

        # tempo da última mudança de estado da bateria
        self.last_time_battery_state_changed_ = self.get_current_time_seconds() 

        # criar um timer que chama check_battery_state a cada 0.1 segundos
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state) 

        # criar um cliente para o serviço "set_led"
        self.set_led_client_ = self.create_client(SetLed, "set_led") 
        
        self.get_logger().info("Battery node has been started.")

    # função para obter o tempo atual em segundos
    def get_current_time_seconds(self): 
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0
    
    # função para checar o estado da bateria e alternar entre cheio e vazio
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging...")
                self.call_set_led(2, 1)
                self.last_time_battery_state_changed_ = time_now
        elif self.battery_state_ == "empty":
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is now full.")
                self.call_set_led(2, 0)
                self.last_time_battery_state_changed_ = time_now

    # função para chamar o serviço set_led
    def call_set_led(self, led_number, state):
        while not self.set_led_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set Led service")
        
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.set_led_client_.call_async(request)
        future.add_done_callback(self.callback_call_set_led)

    # função de callback que processa a resposta do serviço set_led
    def callback_call_set_led(self, future):
        response: SetLed.Response = future.result()
        if response.success:
            self.get_logger().info("LED state was changed")
        else:
            self.get_logger().info("LED not changed")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()