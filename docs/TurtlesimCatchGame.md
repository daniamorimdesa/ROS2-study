# Projeto Turtlesim Catch Them All

## 🎓 Contexto do Curso

Este projeto foi desenvolvido como **projeto final** do curso **"ROS2 for Beginners (ROS Jazzy - 2025)"** da **Udemy**. Representa a culminação do aprendizado, integrando todos os conceitos fundamentais de ROS2 em uma aplicação prática e completa.

## 🎯 Desafio Original

### Objetivo do Projeto
Desenvolver uma **simulação robótica** onde um robô principal (turtle1) navega autonomamente para interceptar e "capturar" múltiplos alvos que aparecem aleatoriamente no ambiente, utilizando o pacote **Turtlesim** como ferramenta de simulação para visualizar o comportamento do robô.

### Metodologia de Desenvolvimento
O curso propõe uma abordagem estruturada para desenvolvimento de aplicações robóticas:
1. **Design arquitetural** - Planejar sistema antes de implementar
2. **Desenvolvimento incremental** - Funcionalidades por etapas
3. **Validação contínua** - Testar cada componente
4. **Integração de conceitos** - Aplicar conhecimentos em contexto prático

## 📋 Especificações do Desafio

### Estrutura obrigatória: 3 Nós
1. **`turtlesim_node`** (pacote turtlesim) - Simulador de ambiente 2D
2. **`turtle_controller`** (customizado) - Controlador de navegação autônoma
3. **`turtle_spawner`** (customizado) - Gerenciador de alvos dinâmicos

### Funcionalidades Requeridas

#### 🎮 Turtle Spawner Node (Gerenciador de Alvos)
- ✅ Chamar serviço `/spawn` com coordenadas aleatórias (0.0 a 11.0)
- ✅ Chamar serviço `/kill` para remover alvos capturados
- ✅ Publicar lista de alvos ativos em `/alive_turtles`
- ✅ Implementar servidor de serviço para processar capturas

#### 🎯 Turtle Controller Node (Sistema de Navegação)
- ✅ Loop de controle com timer de alta frequência
- ✅ Controlar robô via `/turtle1/pose` e `/turtle1/cmd_vel`
- ✅ Implementar **Controlador P** para navegação autônoma
- ✅ Subscrever `/alive_turtles` para seleção de alvos
- ✅ Chamar serviço `/catch_turtle` ao interceptar alvos

### Interfaces Customizadas Obrigatórias
- **`Turtle.msg`** - Estrutura de dados para posição e identificação de alvos
- **`TurtleArray.msg`** - Lista de alvos ativos para `/alive_turtles`
- **`CatchTurtle.srv`** - Protocolo de comunicação para notificar interceptações

### Parâmetros de Configuração
- `/turtle_controller/catch_closest_turtle_first` - Estratégia de seleção de alvos
- `/turtle_spawner/spawn_frequency` - Taxa de geração de novos alvos
- `/turtle_spawner/turtle_name_prefix` - Convenção de nomenclatura

## 📊 Diagrama RQT Graph Esperado
```
┌─────────────────┐    /turtle1/pose     ┌──────────────────┐
│  turtlesim_node │◄────────────────────►│ turtle_controller│
│                 │    /turtle1/cmd_vel  │                  │
└─────────────────┘                      └──────────────────┘
         ▲                                         │
         │ /spawn, /kill                          │ /catch_turtle
         │                                         ▼
┌─────────────────┐    /alive_turtles    ┌──────────────────┐
│ turtle_spawner  │◄────────────────────►│ turtle_controller│
│                 │                      │                  │
└─────────────────┘                      └──────────────────┘
```

## 🏗️ Etapas de Desenvolvimento (Curso)

### **Step 1** - Controle Básico
- ✅ Criar `turtle_controller`
- ✅ Subscrever `/turtle1/pose`
- ✅ Implementar loop de controle para alvo fixo
- ✅ Calcular distâncias e ângulos (matemática)
- ✅ Publicar comandos em `/turtle1/cmd_vel`

### **Step 2** - Spawn de Tartarugas
- ✅ Criar `turtle_spawner`
- ✅ Timer para spawn periódico
- ✅ Chamar serviço `/spawn` com coordenadas aleatórias

### **Step 3** - Sistema de Comunicação
- ✅ Array de alvos ativos no `turtle_spawner`
- ✅ Publicar dados em `/alive_turtles`
- ✅ `turtle_controller` subscreve e seleciona alvos

### **Step 4** - Protocolo de Interceptação
- ✅ Criar serviço `/catch_turtle` no `turtle_spawner`
- ✅ `turtle_controller` notifica interceptações via serviço
- ✅ `turtle_spawner` remove alvos e atualiza estado do sistema

### **Step 5** - Otimização de Estratégia
- ✅ Selecionar alvo **mais próximo** em vez do primeiro da lista

### **Step 6** - Configuração de Sistema
- ✅ Adicionar parâmetros ROS2 para flexibilidade
- ✅ Criar launch file + arquivo YAML para deployment
- ✅ Sistema completo pronto para produção

## Arquitetura do Sistema

### 📦 Pacotes Envolvidos

1. **`turtlesim_catch_them_all`** - Pacote principal com a lógica do jogo
2. **`my_robot_interfaces`** - Interfaces customizadas (mensagens e serviços)
3. **`my_robot_bringup`** - Arquivos de launch
4. **`turtlesim`** - Simulador de tartarugas (pacote padrão do ROS2)

### 🎯 Componentes Principais

#### 1. **Turtle Spawner Node** (`turtle_spawner`)
- **Função**: Gera novas tartarugas aleatoriamente a cada 2 segundos
- **Arquivo**: `turtle_spawner.py` ou `turtle_spawner_course.py`
- **Responsabilidades**:
  - Criar tartarugas em posições aleatórias
  - Manter lista de tartarugas vivas
  - Publicar lista atualizada de tartarugas
  - Remover tartarugas quando capturadas

#### 2. **Turtle Controller Node** (`turtle_controller`)
- **Função**: Controla a turtle1 para perseguir e capturar outras tartarugas
- **Arquivo**: `turtle_controller.py` ou `turtle_controller_course.py`
- **Responsabilidades**:
  - Receber posição atual da turtle1
  - Calcular trajetória até a tartaruga alvo
  - Enviar comandos de movimento
  - Solicitar captura quando próximo do alvo

#### 3. **Turtlesim Node** (`turtlesim_node`)
- **Função**: Simulador gráfico das tartarugas
- **Pacote**: `turtlesim` (padrão do ROS2)
- **Responsabilidades**:
  - Renderizar interface gráfica
  - Gerenciar movimento das tartarugas
  - Fornecer serviços de spawn/kill

## 🔄 Fluxo de Comunicação

### Tópicos (Topics)

| Tópico | Tipo | Publisher | Subscriber | Descrição |
|--------|------|-----------|------------|-----------|
| `/turtle1/pose` | `turtlesim/msg/Pose` | turtlesim_node | turtle_controller | Posição atual da turtle1 |
| `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | turtle_controller | turtlesim_node | Comandos de movimento |
| `alive_turtles` | `my_robot_interfaces/msg/TurtleArray` | turtle_spawner | turtle_controller | Lista de tartarugas vivas |

### Serviços (Services)

| Serviço | Tipo | Server | Client | Descrição |
|---------|------|--------|--------|-----------|
| `/spawn` | `turtlesim/srv/Spawn` | turtlesim_node | turtle_spawner | Criar nova tartaruga |
| `/kill` | `turtlesim/srv/Kill` | turtlesim_node | turtle_spawner | Remover tartaruga |
| `catch_turtle` | `my_robot_interfaces/srv/CatchTurtle` | turtle_spawner | turtle_controller | Notificar captura |

## 🧮 Algoritmo de Controle

### Controle de Posição
```python
# Calcular distância até o alvo
dist_x = target.x - current.x
dist_y = target.y - current.y
distance = sqrt(dist_x² + dist_y²)

# Velocidade linear proporcional à distância
cmd.linear.x = K_linear * distance
```

### Controle de Orientação
```python
# Calcular ângulo desejado
goal_theta = atan2(dist_y, dist_x)

# Normalizar diferença angular
diff = goal_theta - current_theta
if diff > π: diff -= 2π
elif diff < -π: diff += 2π

# Velocidade angular proporcional ao erro
cmd.angular.z = K_angular * diff
```

### Detecção de Captura
```python
if distance < 0.5:  # Threshold de captura
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    call_catch_turtle_service(target_name)
```

## 📋 Interfaces Customizadas

### Mensagem: `Turtle`
```
string name
float64 x
float64 y
float64 theta
```

### Mensagem: `TurtleArray`
```
Turtle[] turtles
```

### Serviço: `CatchTurtle`
```
# Request
string name
---
# Response
bool success
```

## 🚀 Como Executar

### 1. Build do Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Executar com Launch File
```bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.xml
```

### 3. Executar Nós Individualmente
```bash
# Terminal 1 - Simulador
ros2 run turtlesim turtlesim_node

# Terminal 2 - Spawner
ros2 run turtlesim_catch_them_all turtle_spawner

# Terminal 3 - Controller
ros2 run turtlesim_catch_them_all turtle_controller
```

## 🔧 Parâmetros Ajustados

### Turtle Spawner
- **Intervalo de spawn**: 2.0 segundos (timer)
- **Área de spawn**: x=[2.0, 10.0], y=[2.0, 10.0]
- **Prefixo dos nomes**: "turtle"

### Turtle Controller
- **Frequência de controle**: 100 Hz (0.01s timer)
- **Ganho linear**: K_linear = 2.0
- **Ganho angular**: K_angular = 6.0
- **Threshold de captura**: 0.5 unidades

## 🤖 Comportamento da Simulação

1. **Inicialização**: Robô principal (turtle1) inicia no centro do ambiente
2. **Geração de Alvos**: Novos alvos aparecem automaticamente a cada 2 segundos
3. **Navegação Autônoma**: Robô se move automaticamente em direção ao alvo selecionado
4. **Interceptação**: Quando robô se aproxima (< 0.5 unidades), o alvo é "capturado" e removido
5. **Ciclo Contínuo**: Processo repete indefinidamente, simulando missões de coleta/interceptação

## 🛠️ Conceitos ROS2 Demonstrados

Este projeto é uma **implementação completa** que demonstra:

- ✅ **Nós** (Nodes): Modularização em turtle_spawner e turtle_controller
- ✅ **Tópicos** (Topics): Comunicação assíncrona para poses e comandos
- ✅ **Serviços** (Services): Chamadas síncronas para spawn/kill/catch
- ✅ **Mensagens Customizadas**: Turtle e TurtleArray
- ✅ **Serviços Customizados**: CatchTurtle
- ✅ **Timers**: Loops periódicos para spawn e controle
- ✅ **Callbacks Assíncronos**: Processamento não-bloqueante
- ✅ **Launch Files**: Inicialização coordenada de múltiplos nós
- ✅ **Controle de Robôs**: Algoritmos de navegação e controle
- ✅ **Parâmetros ROS2**: Configuração dinâmica do sistema
- ✅ **Arquivos YAML**: Configuração externa de parâmetros

## 🎯 Valor Educativo do Curso

### Habilidades Desenvolvidas
- **Arquitetura de Sistemas**: Design de aplicações multi-nó
- **Comunicação ROS2**: Padrões pub/sub e client/server
- **Matemática Robótica**: Controle proporcional e navegação
- **Interfaces Customizadas**: Criação de mensagens e serviços
- **Organização de Projetos**: Estrutura multi-pacote
- **Debugging**: Solução de problemas reais
- **Metodologia**: Desenvolvimento incremental

### Progressão do Aprendizado
1. **Conceitos Básicos** → **Aplicação Prática**
2. **Nós Isolados** → **Sistema Integrado**  
3. **Funcionalidades Simples** → **Comportamento Complexo**
4. **Código Hardcoded** → **Configuração Flexível**

## 📚 Aprendizados Específicos

### Desenvolvimento Incremental
- Como quebrar um projeto complexo em etapas simples
- Importância do design antes da implementação
- Valor da validação contínua durante desenvolvimento

### Padrões ROS2
- **Publisher/Subscriber**: Para dados contínuos (pose, cmd_vel)
- **Service Client/Server**: Para ações pontuais (spawn, kill, catch)
- **Timer Callbacks**: Para loops de controle periódicos
- **Async Callbacks**: Para responsividade do sistema

### Controle de Robôs
- **Controlador P**: Implementação prática de controle proporcional
- **Normalização Angular**: Tratamento correto de ângulos (-π a π)
- **Threshold de Proximidade**: Detecção de eventos espaciais

