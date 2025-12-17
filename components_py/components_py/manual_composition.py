#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from components_py.node1 import Node1
from components_py.node2 import Node2

"""Este exemplo demonstra a composição manual de múltiplos nós em um único executor.
Usamos o SingleThreadedExecutor para gerenciar a execução do Node1 e Node2.
Cada nó possui múltiplos timers que simulam callbacks demorados usando time.sleep().
Ao executar este script, você verá que os callbacks dos timers são chamados de forma sequencial,
devido à natureza do SingleThreadedExecutor.
"""

def main(args=None):
    rclpy.init(args=args)

    node1 = Node1()
    node2 = Node2()

    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()