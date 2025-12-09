#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from example_interfaces.msg import Int64
#----------------------------------------------------------------------------------------------------------------
# versão usando LifecycleNode
#----------------------------------------------------------------------------------------------------------------
class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher") # criar um nó chamado "number_publisher"
        self.get_logger().info("In constructor...")
        self.number_ = 1                     # iniciar o número em 1
        self.publish_frequency_ = 1.0        # frequência de publicação em Hz
        self.number_publisher_ = None        # inicializar o publicador como None
        self.number_timer_ = None            # inicializar o timer como None

    # Criar as comunicações ROS2, conectar ao hardware (create ROS2 communications, connect to HW)
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("In on_configure...")  

        # criar um publicador para o tópico "number"
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10) 

        # criar um timer para publicar os números periodicamente
        self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_,self.publish_number)

        # cancelar o timer para que ele não publique até que o nó esteja ativo
        self.number_timer_.cancel()

        return TransitionCallbackReturn.SUCCESS # indicar que a configuração foi bem-sucedida
    
    # Ativar as comunicações ROS2, ativar/habilitar hardware (activate ROS2 communications, activate/enable HW)
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate...")

        # reiniciar o timer para começar a publicar números
        self.number_timer_.reset() 

        return super().on_activate(previous_state)

    # Desativar as comunicações ROS2, desativar/desabilitar hardware (deactivate ROS2 communications, deactivate/disable HW)
    def on_deactivate(self, previous_state: LifecycleState): 
        self.get_logger().info("In on_deactivate...")

        # cancelar o timer para parar de publicar números
        self.number_timer_.cancel()

        return super().on_deactivate(previous_state)    
    
    # Destruir as comunicações ROS2, desconectar do hardware (destroy ROS2 communications, disconnect from HW)
    def on_cleanup(self, previous_state: LifecycleState): 
        self.get_logger().info("In on_cleanup...")

        self.destroy_timer(self.number_timer_)                   # destruir o timer
        self.destroy_lifecycle_publisher(self.number_publisher_) # destruir o publicador
        
        # self.number_timer_ = None                                # limpar a referência do timer
        # self.number_publisher_ = None                            # limpar a referência do publicador

        return TransitionCallbackReturn.SUCCESS                  # indicar que a limpeza foi bem-sucedida
    
    def on_shutdown(self, previous_state: LifecycleState): 
        self.get_logger().info("In on_shutdown...") 

        self.destroy_timer(self.number_timer_)                   # destruir o timer
        self.destroy_lifecycle_publisher(self.number_publisher_) # destruir o publicador
        
        # self.number_timer_ = None                                # limpar a referência do timer
        # self.number_publisher_ = None                            # limpar a referência do publicador

        return TransitionCallbackReturn.SUCCESS                  # indicar que a limpeza foi bem-sucedida
    
    # publicar o número atual (callback do timer)
    def publish_number(self): 
        msg = Int64()                                                    # criar uma mensagem do tipo Int64
        msg.data = self.number_                                          # definir o valor do número na mensagem
        self.number_publisher_.publish(msg)                              # publicar a mensagem no tópico
        self.number_ += 1                                                # incrementar o número para a próxima publicação

def main(args=None): 
    rclpy.init(args=args)                  # inicializar o rclpy
    node = NumberPublisherNode()           # criar uma instância do nó NumberPublisherNode
    rclpy.spin(node)                       # manter o nó em execução
    rclpy.shutdown()                       # desligar o rclpy




#----------------------------------------------------------------------------------------------------------------
# versão normal
#----------------------------------------------------------------------------------------------------------------
#from rclpy.node import Node

# class NumberPublisherNode(Node):
#     def __init__(self):
#         super().__init__("number_publisher") # criar um nó chamado "number_publisher"
#         self.number_ = 1                     # iniciar o número em 1
#         self.publish_frequency_ = 1.0        # frequência de publicação em Hz
#         self.number_publisher_ = self.create_publisher(Int64, "number", 10) # criar um publicador para o tópico "number"
#         # criar um timer para publicar os números periodicamente
#         self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_,self.publish_number)
#         self.get_logger().info("Number publisher has been started!")

#     # publicar o número atual (callback do timer)
#     def publish_number(self): 
#         msg = Int64()                                                    # criar uma mensagem do tipo Int64
#         msg.data = self.number_                                          # definir o valor do número na mensagem
#         self.number_publisher_.publish(msg)                              # publicar a mensagem no tópico
#         self.get_logger().info(f"Published number: {str(self.number_)}") # registrar o número publicado
#         self.number_ += 1                                                # incrementar o número para a próxima publicação

# def main(args=None): 
#     rclpy.init(args=args)                  # inicializar o rclpy
#     node = NumberPublisherNode()           # criar uma instância do nó NumberPublisherNode
#     rclpy.spin(node)                       # manter o nó em execução
#     rclpy.shutdown()                       # desligar o rclpy
#----------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    main()
