#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = Node("add_two_ints_client_no_oop") # instanciar o nó

    client = node.create_client(AddTwoInts, "add_two_ints") # criar um cliente de serviço chamado "add_two_ints" que usa o tipo AddTwoInts
    while not client.wait_for_service(1.0): # esperar até que o serviço esteja disponível
        node.get_logger().warn("Waiting for Add Two Ints server...") # registrar uma mensagem de log

    request = AddTwoInts.Request() # criar uma requisição do tipo AddTwoInts.Request
    request.a = 3  # definir o valor do primeiro inteiro
    request.b = 7  # definir o valor do segundo inteiro

    future = client.call_async(request) # chamar o serviço de forma assíncrona e obter um futuro
    rclpy.spin_until_future_complete(node, future) # esperar até que o futuro esteja completo

    response = future.result() # obter o resultado da resposta
    node.get_logger().info(str(request.a)+ " + " + str(request.b) + " = " + str(response.sum)) # registrar a soma obtida na resposta

    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()