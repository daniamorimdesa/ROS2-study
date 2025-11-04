# ROS2: Criando Interfaces Personalizadas (msg e srv)

## 1. Criar um package de interfaces
```bash
ros2 pkg create my_robot_interfaces
```

Mais detalhes e exemplos: [roboticsbackend.com/ros2-create-custom-message/](https://roboticsbackend.com/ros2-create-custom-message/)

## 2. Criar uma mensagem personalizada (msg)
1. Crie a pasta `msg` dentro do seu package de interfaces:
   ```bash
   mkdir my_robot_interfaces/msg
   ```
2. Crie um arquivo de mensagem (ex: `HardwareStatus.msg`) com nome em CamelCase:
   ```bash
   nano my_robot_interfaces/msg/HardwareStatus.msg
   ```
3. Preencha com os campos desejados, exemplo:
   ```
   # HardwareStatus.msg: Message definition for hardware status of a robot
   float64 temperature
   bool are_motors_ready
   string debug_message
   ```

## 3. Editar o CMakeLists.txt
Adicione as mensagens criadas ao comando `rosidl_generate_interfaces`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

## 4. Editar o package.xml
Inclua as dependências necessárias:
```xml
<depend>rclpy</depend>
<depend>example_interfaces</depend>
<depend>my_robot_interfaces</depend>
```

## 5. Instalar dependências para build
Para ROS2 Humble, garanta que estes pacotes Python estejam instalados:
```bash
pip install empy==3.3.4 catkin_pkg lark
```

## 6. Buildar o workspace
```bash
colcon build
```

## 7. Usar e visualizar a interface
Abra um novo terminal, execute:
```bash
source ~/.bashrc
ros2 interface show my_robot_interfaces/msg/HardwareStatus
```

Se o VS Code não enxergar a interface, feche e reabra o programa a partir do terminal com o ambiente já "sourced".

## 8. Listar interfaces
- Ver todas as interfaces disponíveis:
  ```bash
  ros2 interface list
  ```
- Ver interfaces de um package específico:
  ```bash
  ros2 interface package my_robot_interfaces
  ```

## 9. Criar um serviço personalizado (srv)
1. Crie a pasta `srv` dentro do package de interfaces:
   ```bash
   mkdir my_robot_interfaces/srv
   ```
2. Crie o arquivo do serviço, exemplo:
   ```bash
   nano my_robot_interfaces/srv/ComputeRectangleArea.srv
   ```
3. Defina request e response:
   ```
   float64 length
   float64 width
   ---
   float64 area
   ```
4. Inclua no `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/HardwareStatus.msg"
     "srv/ComputeRectangleArea.srv"
   )
   ```

---

Com essas etapas, você pode criar e usar mensagens e serviços personalizados no seu projeto ROS2!
