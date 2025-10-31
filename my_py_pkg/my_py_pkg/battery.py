#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLED
from functools import partial
import time
#-------------------------------------------------------------------------------------------------------------------------------------------
# Step 1: create LED panel node and publish the LED panel state (with a custom message)
# Step 2: add a service server inside the LED panel node (with a custom service definition)
# Step 3: create the battery node, simulate the battery life, and call the “set_led” service with a service client
#-------------------------------------------------------------------------------------------------------------------------------------------

class Battery(Node):

    def __init__(self):
        super().__init__("battery") # criar um nó chamado "battery", client do service set_led
        self.client_ = self.create_client(SetLED, "set_led") # criar um cliente de serviço chamado "set_led" que usa o tipo SetLED

    def call_set_led(self, led_number, led_state): # função para chamar o serviço de setar led
        while not self.client_.wait_for_service(1.0): # esperar até que o serviço esteja disponível
            self.get_logger().warn("Waiting for Set LED server...") # registrar uma mensagem de log enquanto espera
        request = SetLED.Request() # criar uma requisição do tipo SetLED
        # definir os valores da requisição
        request.led_number = led_number 
        request.led_state = led_state

        future = self.client_.call_async(request) # chamar o serviço de forma assíncrona

        # adicionar uma função de callback para processar a resposta quando estiver pronta
        # se quiser passar argumentos adicionais para o callback, use partial
        future.add_done_callback(partial(self.callback_call_set_led, request=request))
    
    def callback_call_set_led(self, future, request): # função de callback que processa a resposta do serviço
        response = future.result()                    # obter o resultado da resposta
        #self.get_logger().info(f"Led number: {request.led_number} | State: {request.led_state} | Result: {response.success}") # registrar o resultado da chamada do serviço


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = Battery() # instanciar o nó

    # simular a vida da bateria e chamar o serviço para atualizar o estado do LED
    # a bateria começa cheia, fica vazia após 4 segundos, e depois cheia novamente após mais 6 segundos, fica nesse loop
    # quando cheia, o estado é off, quando vazia, on
    while rclpy.ok():
        time.sleep(4)                             # esperar 4 segundos
        node.call_set_led(2, "on")                # ligar o LED 3
        node.get_logger().info("Battery empty!")  # registrar que a bateria está vazia
        time.sleep(6)                             # esperar 6 segundos
        node.call_set_led(2, "off")               # desligar o LED 3
        node.get_logger().info("Battery full!")   # registrar que a bateria está cheia

    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()