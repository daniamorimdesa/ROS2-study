#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time

"""
Este exemplo demonstra o uso do MultiThreadedExecutor em ROS2:

- ReentrantCallbackGroup: Permite que múltiplos callbacks sejam executados simultaneamente em diferentes threads.
  Isso é útil quando os callbacks não compartilham recursos e podem ser executados em paralelo

- MutuallyExclusiveCallbackGroup: Garante que apenas um callback dentro do grupo seja executado por vez.
    Isso é útil quando os callbacks compartilham recursos e precisam ser protegidos contra acesso concorrente.
"""

class Node1(Node):
    def __init__(self):
        super().__init__("node1")

        # criar um callback group reentrante para permitir a execução concorrente dos callbacks
        # self.cb_group_ = ReentrantCallbackGroup()
        # self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group_)
        # self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group_)
        # self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group_)

        # criar um callback group mutuamente exclusivo para garantir que os callbacks não sejam executados simultaneamente
        self.cb_group1_ = MutuallyExclusiveCallbackGroup() # garantir exclusividade para cb1
        # self.cb_group2_ = ReentrantCallbackGroup()         # permitir concorrência entre cb2 e cb3
        self.cb_group2_ = MutuallyExclusiveCallbackGroup()   # garantir exclusividade para cb2 e cb3        
        
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group1_)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group2_)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group2_)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")


class Node2(Node):
    def __init__(self):
        super().__init__("node2")

        self.cb_group_ = ReentrantCallbackGroup() # permitir concorrência entre os callbacks do Node2

        self.timer4_ = self.create_timer(1.0, self.callback_timer4, callback_group=self.cb_group_)
        self.timer5_ = self.create_timer(1.0, self.callback_timer5)


    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node2 = Node2()

    """
    Usando MultiThreadedExecutor, os callbacks podem ser executados em paralelo
    em múltiplos threads.

    Isso significa que se um callback demora muito tempo para ser executado (como
    simulado pelos time.sleep(2.0)), os outros callbacks podem ser executados
    simultaneamente em threads diferentes, reduzindo o tempo de espera geral
    para que todos os callbacks sejam processados.

    Isso pode melhorar significativamente a performance em cenários onde há
    muitos callbacks ou onde os callbacks são demorados. No entanto, é importante 
    notar que o uso de múltiplos threads pode introduzir complexidade adicional, 
    como a necessidade de gerenciar o acesso concorrente a recursos compartilhados, 
    o que pode levar a condições de corrida se não for tratado adequadamente.
    """

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()

    rclpy.shutdown()
    
#----------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()