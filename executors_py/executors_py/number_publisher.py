#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = self.create_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.get_logger().info("Number publisher has been started.")

        # Main callbacks in ROS2:
        # - timers
        # - subscribers
        # - service servers
        # - action servers
        # - futures (in clients)


    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()

    # Forma tradicional de rodar um node em ROS2
    # rclpy.spin(node, SingleThreadedExecutor())


    """
    Um executor em ROS2 é responsável por gerenciar a execução dos nodes e seus callbacks associados.

    Ele lida com a fila de eventos, garantindo que os callbacks sejam chamados quando apropriado, como 
    quando uma mensagem é recebida em um tópico, um timer expira, ou uma solicitação de serviço é feita.

    O executor pode ser configurado para operar em um único thread (SingleThreadedExecutor) ou em múltiplos 
    threads (MultiThreadedExecutor), dependendo das necessidades da aplicação.
    """
    
    # Forma alternativa de rodar um node em ROS2 usando um executor explicitamente
    executor = SingleThreadedExecutor() # criar um executor, ele vai gerenciar os nodes e callbacks 
    executor.add_node(node)             # adicionar o node ao executor
    executor.spin()                     # iniciar o executor


    rclpy.shutdown()

#----------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()