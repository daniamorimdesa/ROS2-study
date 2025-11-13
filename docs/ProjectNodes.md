# Nós do Workspace

Este documento lista os nós presentes no workspace, com uma breve descrição, tópicos e serviços utilizados, parâmetros importantes e exemplos de execução.

## Resumo rápido

- Pacotes com nós:
  - `turtlesim_catch_them_all` — gerenciador de alvos e controlador de navegação (simulação com turtlesim)
  - `my_py_pkg` — vários exemplos e utilitários em Python (publishers, subscribers, serviços)
  - `my_cpp_pkg` — exemplo de nó C++ simples
  - `turtlesim` — nó do simulador (`turtlesim_node`)

---

## turtlesim_catch_them_all

Nó(s):

- `turtle_spawner` (executable: `turtle_spawner`)
  - Pacote: `turtlesim_catch_them_all`
  - Tipo: Server de serviços (client para `/spawn` e `/kill`), publisher
  - Tópicos publicados:
    - `alive_turtles` (`my_robot_interfaces/msg/TurtleArray`) — lista de alvos ativos (name,x,y,theta)
  - Serviços usados:
    - ` /spawn` (`turtlesim/srv/Spawn`) — chamado para criar alvos
    - ` /kill` (`turtlesim/srv/Kill`) — chamado para remover alvos
    - `catch_turtle` (`my_robot_interfaces/srv/CatchTurtle`) — server próprio para receber pedidos de captura
  - Parâmetros: (opcionalmente presentes em variantes) `spawn_frequency`, `turtle_name_prefix`
  - Descrição: Gera alvos em posições aleatórias, mantém e publica lista de alvos, processa pedidos de captura removendo alvos e chamando `/kill`.
  - Execução:
    - `ros2 run turtlesim_catch_them_all turtle_spawner`

- `turtle_controller` (executable: `turtle_controller`)
  - Pacote: `turtlesim_catch_them_all`
  - Tipo: Subscriber + publisher + service client
  - Subscrições:
    - `/turtle1/pose` (`turtlesim/msg/Pose`) — posição atual do robô mestre
    - `alive_turtles` (`my_robot_interfaces/msg/TurtleArray`) — lista de alvos ativos
  - Publicações:
    - `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) — comandos de velocidade para navegação
  - Serviços usados:
    - `catch_turtle` (`my_robot_interfaces/srv/CatchTurtle`) — client chamado quando o alvo é interceptado
  - Parâmetros: `catch_closest_turtle_first` (se implementado)
  - Descrição: Loop de controle que calcula distância e ângulo até o alvo selecionado e publica comandos de velocidade (Controlador P simplificado).
  - Execução:
    - `ros2 run turtlesim_catch_them_all turtle_controller`

- Variantes (didáticas/curso): `turtle_spawner_course`, `turtle_controller_course`
  - Executáveis: `turtle_spawner_course`, `turtle_controller_course`
  - Mesmo papel funcional que os nós principais, mas com comentários/versões pedagógicas. Podem aparecer como entry points no `setup.py`.
  - Execução:
    - `ros2 run turtlesim_catch_them_all turtle_spawner_course`
    - `ros2 run turtlesim_catch_them_all turtle_controller_course`

---

## my_py_pkg

Este pacote contém muitos exemplos em Python. Abaixo estão os executáveis (conforme `setup.py`) e um resumo rápido de cada um — os mais relevantes para integração estão com mais detalhes.

- `py_node` — exemplo básico (arquivo `my_first_node.py`).
- `robot_news_station` — publisher/subscriber demonstrando mensagens e timers.
- `smartphone` — exemplo de cliente/integração (simula um dispositivo remoto).
- `number_publisher` — publica um número no tópico `number` (exemplo de parâmetros dinâmicos).
  - Tópico publicado: `number` (`example_interfaces/msg/Int64`)
  - Parâmetros:
    - `number` (int, padrão 2)
    - `timer_period` (float, período do timer em s)
  - Suporta alteração de parâmetro em runtime via callback.
  - Execução: `ros2 run my_py_pkg number_publisher`

- `number_counter` — subscreve `number`, acumula soma e publica `number_count`; também expõe serviço `reset_counter` (`example_interfaces/srv/SetBool`).
  - Subscrição: `number` (`Int64`)
  - Publicação: `number_count` (`Int64`)
  - Serviço: `reset_counter` (`SetBool`) — reseta o contador quando chamado com `true`.
  - Execução: `ros2 run my_py_pkg number_counter`

- `add_two_ints_server` — servidor de serviço de soma (exemplo de serviço)
- `add_two_ints_client_no_oop` — cliente de serviço (versão sem OOP)
- `add_two_ints_client` — cliente de serviço (versão com OOP)
- `reset_counter` — nó que expõe o serviço `reset_counter` (separado)
- `hardware_status_publisher` — publisher que simula status de hardware
- `led_panel` — simulação de painel de LEDs (publisher/subscriber)
- `battery` — simulação de estado de bateria

> Observação: para ver a implementação completa de cada nó Python, abra os respectivos arquivos em `src/my_py_pkg/my_py_pkg/`.

---

## my_cpp_pkg

- Executable: `cpp_node` (instalado como `lib/my_cpp_pkg/cpp_node`)
  - Arquivo fonte: `src/my_first_node.cpp`
  - Descrição: Exemplo simples em C++ que imprime mensagens periódicas (timer) e incrementa um contador.
  - Execução:
    - Após `colcon build` e `source install/setup.bash` execute:
      `ros2 run my_cpp_pkg cpp_node`

---

## turtlesim (sistema)

- `turtlesim_node` — nó do pacote `turtlesim` que simula o ambiente 2D.
  - Tópicos relevantes:
    - `/turtle1/pose` (`turtlesim/msg/Pose`) — posição e orientação
    - `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) — comando de velocidade
  - Serviços:
    - `/spawn` (`turtlesim/srv/Spawn`)
    - `/kill` (`turtlesim/srv/Kill`)
  - Execução: `ros2 run turtlesim turtlesim_node`

---

## Como rodar o sistema completo

1. Build e source do workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. Rodar via launch (exemplo presente em `my_robot_bringup`)

```bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.xml
```

3. Rodar nós individuais (exemplos)

```bash
# Simulador
ros2 run turtlesim turtlesim_node

# Spawner (alvos)
ros2 run turtlesim_catch_them_all turtle_spawner

# Controller (navegação)
ros2 run turtlesim_catch_them_all turtle_controller

# Exemplo Python publisher/counter
ros2 run my_py_pkg number_publisher
ros2 run my_py_pkg number_counter

# Nó C++ exemplo
ros2 run my_cpp_pkg cpp_node
```

---

## Notas finais

- Use `ros2 node list`, `ros2 topic list`, `ros2 service list` e `ros2 node info <node>` para inspecionar o comportamento em runtime.
- Se quiser, posso adicionar uma tabela resumida (CSV/Markdown) dos nós com colunas: `node name`, `package`, `executable`, `topics`, `services`, `parameters`.
