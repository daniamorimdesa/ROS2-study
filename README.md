# Guia rápido: Criar um Node Python no ROS2

## 1. Criar o arquivo do node

Lembrar de estar na pasta correta(exemplo):
```bash
cd ros2_ws/src/my_py_pkg/my_py_pkg
```
Criar o arquivo.py: 
```bash
touch nome_do_node.py
```

## 2. Tornar o arquivo executável
```bash
chmod +x nome_do_node.py
```

## 3. Escrever o código do node
- Importe `rclpy` e crie sua classe/função principal.
- Exemplo básico:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station") # criar um nó chamado "robot_news_station"
        self.robot_name = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # criar um publicador no tópico "robot_news" com fila de tamanho 10
        self.timer_ = self.create_timer(0.5, self.publish_news) # criar um timer que chama publish_news a cada 0.5 segundos
        self.get_logger().info("Robot News has been started!")  # registrar uma mensagem de log

    def publish_news(self):
        msg = String() # criar uma mensagem do tipo String
        msg.data = f"Hello, this is {self.robot_name} from the robot news station :)"
        self.publisher_.publish(msg) # publicar a mensagem


def main(args=None):
    rclpy.init(args=args) # inicializar o rclpy
    node = RobotNewsStationNode() # instanciar o nó
    rclpy.spin(node) # manter o nó ativo para processar callbacks(até que se aperte Ctrl+C)
    rclpy.shutdown() # finalizar o rclpy
#-----------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
```

## 4. Incluir o node no `setup.py`
No campo `entry_points`, adicione:
```python
'console_scripts': [
    'nome_do_executavel = my_py_pkg.nome_do_node:main',
],
```

## 5. Checar dependências no `package.xml`
- Se usou bibliotecas novas, adicione-as como `<exec_depend>nome_da_dependencia</exec_depend>`

## 6. Compilar o workspace
```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
```

## 7. Fonte no ambiente
```bash
source install/setup.bash
```

## 8. Rodar o node
```bash
ros2 run my_py_pkg nome_do_executavel
```

---

> Dica: Sempre cheque permissões e se o node está listado corretamente no `setup.py`.


---

## Comandos úteis ROS2

### Listar nodes em execução
```bash
ros2 node list
```

### Listar tópicos disponíveis
```bash
ros2 topic list
```

### Ver detalhes de um tópico (tipo, publishers, subscribers)
```bash
ros2 topic info /nome_do_topico
```

### Visualização gráfica dos nodes/tópicos
```bash
rqt_graph
```

### Rodar um publisher (exemplo)
```bash
ros2 run my_py_pkg robot_news_station
```

### Escutar mensagens de um tópico (subscriber no terminal), exemplo:
```bash
ros2 topic echo /robot_news
```

### Ver frequência (Hz) de publicação de mensagens (exemplo):
```bash
ros2 topic hz /robot_news
```

### Ver largura de banda (bandwidth) de um tópico (exemplo):
```bash
ros2 topic bw /robot_news
```

### Renomear um arquivo no terminal(exemplo):
```bash
ros2 run my_py_pkg py_node --ros-args -r __node:=abc
```

### Renomear um tópico em tempo real (exemplo):
No lançamento do node:
```bash
ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=abc
```

### Ver informação da mensagem de um tópico
```bash
ros2 topic info /robot_news
```

### Publicar mensagem direto do terminal (exemplo com TurtleSim):
1. Descubrir o tipo da mensagem:
```bash
ros2 interface show geometry_msgs/msg/Twist
```
2. Publique uma mensagem:
```bash
ros2 topic pub -r 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.0}}"
```
