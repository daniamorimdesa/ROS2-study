# TIAgo Examples

Este pacote contém exemplos de código desenvolvidos em sala de aula para o robô **TIAgo** da PAL Robotics.

## Descrição

Os códigos neste pacote foram criados como exercícios práticos para aprender a trabalhar com o robô TIAgo, explorando conceitos de navegação, sensores laser e controle de movimento.

## Nodes Disponíveis

### 1. tiago

**Descrição**: Node básico para controlar o movimento do TIAgo até uma posição desejada.

**Funcionalidade**:
- Recebe informações de pose (posição e orientação)
- Move o robô para uma posição desejada
- Realiza rotação até alinhar com o destino
- Move-se em linha reta até o objetivo

**Tópicos**:
- **Subscriber**: `/turtle1/pose` (Pose) - posição atual do robô
- **Publisher**: `/turtle1/cmd_vel` (Twist) - comandos de velocidade

**Executar**:
```bash
ros2 run tiago_examples tiago
```

---

### 2. tiago2

**Descrição**: Versão melhorada do node de controle do TIAgo usando odometria.

**Funcionalidade**:
- Usa dados de odometria em vez de pose simples
- Converte quaternions para ângulos de Euler usando `tf_transformations`
- Controla movimento do robô baseado em coordenadas odométricas
- Implementa lógica de rotação seguida de movimento linear

**Tópicos**:
- **Subscriber**: `/ground_truth_odom` (Odometry) - odometria do robô
- **Publisher**: `/cmd_vel` (Twist) - comandos de velocidade

**Executar**:
```bash
ros2 run tiago_examples tiago2
```

**Notas**:
- Requer `tf_transformations` para conversão de quaternions
- Usa `/ground_truth_odom` para obter pose precisa do robô

---

### 3. laser_tiago

**Descrição**: Node para processar dados do sensor laser do TIAgo e detectar obstáculos.

**Funcionalidade**:
- Lê dados do sensor laser (LaserScan)
- Identifica o obstáculo mais próximo
- Calcula distância e posição (coordenadas cartesianas) do obstáculo
- Exibe informações no console

**Tópicos**:
- **Subscriber**: `/scan_raw` (LaserScan) - dados do sensor laser

**Executar**:
```bash
ros2 run tiago_examples laser_tiago
```

**Saída Exemplo**:
```
Closest obstacle at x: 1.23 m, y: 0.45 m
Distance to closest obstacle: 1.31 m
```

**Notas**:
- Converte coordenadas polares (distância, ângulo) para cartesianas (x, y)
- Útil para detecção de obstáculos e navegação autônoma

---

## Dependências

Este pacote depende de:
- `rclpy` - Biblioteca cliente Python para ROS2
- `geometry_msgs` - Mensagens de geometria (Twist)
- `sensor_msgs` - Mensagens de sensores (LaserScan)
- `nav_msgs` - Mensagens de navegação (Odometry)
- `tf_transformations` - Transformações TF (conversão quaternion/euler)

## Instalação

```bash
cd ~/ros2_ws/src
# O pacote já está instalado

# Compilar
cd ~/ros2_ws
colcon build --packages-select tiago_examples

# Source
source install/setup.bash
```

## Conceitos Aprendidos

### Odometria
- Uso de mensagens `Odometry` para obter pose do robô
- Conversão de quaternions para ângulos de Euler
- Cálculo de distância e ângulo até o objetivo

### Sensor Laser
- Processamento de dados `LaserScan`
- Conversão de coordenadas polares para cartesianas
- Detecção do obstáculo mais próximo

### Controle de Movimento
- Publicação de comandos `Twist` para controlar o robô
- Separação de manobras: rotação seguida de movimento linear
- Uso de flags para controlar estados da máquina de estados

## Robô TIAgo

O **TIAgo** é um robô de serviço móvel desenvolvido pela PAL Robotics, equipado com:
- Base móvel diferencial
- Sensor laser para navegação
- Braço manipulador
- Câmeras e sensores diversos

Mais informações: [PAL Robotics - TIAgo](https://pal-robotics.com/robots/tiago/)


