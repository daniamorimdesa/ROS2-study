# ROS2 common_interfaces: Mensagens e Serviços Padrão

## O que é o common_interfaces?
O pacote [common_interfaces](https://github.com/ros2/common_interfaces) reúne as definições de mensagens e serviços mais comuns e reutilizados no ecossistema ROS2. Ele inclui pacotes como `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, entre outros.

Essas interfaces são essenciais para a comunicação entre nós, pois padronizam os tipos de dados usados em tópicos e serviços.

## Principais pacotes e exemplos

### std_msgs
Mensagens genéricas para comunicação básica.
- `std_msgs/msg/String`
- `std_msgs/msg/Int32`
- `std_msgs/msg/Float64`

### geometry_msgs
Mensagens para posições, vetores e transformações.
- `geometry_msgs/msg/Point`
- `geometry_msgs/msg/Pose`
- `geometry_msgs/msg/Twist`

### sensor_msgs
Mensagens para sensores comuns.
- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/LaserScan`
- `sensor_msgs/msg/Imu`

### nav_msgs
Mensagens para navegação e mapas.
- `nav_msgs/msg/Odometry`
- `nav_msgs/msg/Path`

### example_interfaces
Mensagens e serviços de exemplo para aprendizado e testes.
- `example_interfaces/msg/Int64`
- `example_interfaces/srv/AddTwoInts`

## Como consultar a interface de uma mensagem ou serviço

Veja a estrutura de uma mensagem:
```bash
ros2 interface show std_msgs/msg/String
```
Veja a estrutura de um serviço:
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Exemplos práticos

### Publicar uma mensagem String
```bash
ros2 topic pub /mensagem std_msgs/msg/String "data: 'Olá, ROS2!'"
```

### Publicar velocidade para o turtlesim
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

### Chamar serviço de soma
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

## Links úteis
- [Repositório oficial common_interfaces](https://github.com/ros2/common_interfaces)
- [Documentação das mensagens ROS2](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)

---

Utilize as mensagens e serviços do `common_interfaces` para garantir compatibilidade e facilitar a integração entre diferentes pacotes ROS2!
