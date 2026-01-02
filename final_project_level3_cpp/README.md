# Final Project Level 3 - C++ Lifecycle Component

Este pacote contém a versão C++ do TurtleController como um **Lifecycle Component** que pode ser carregado dinamicamente em um container.

## Compilar o Pacote

```bash
cd ~/ros2_ws
colcon build --packages-select final_project_level3_cpp
source install/setup.bash
```

## Execução com Component Container (3 Terminais)

### Terminal 1: Turtlesim Node
```bash
ros2 run turtlesim turtlesim_node
```

### Terminal 2: Component Container MultiThreaded
```bash
ros2 run rclcpp_components component_container_mt
```

### Terminal 3: Carregar o Componente e Controlar Lifecycle

**1. Carregar o componente no container:**
```bash
ros2 component load /ComponentManager final_project_level3_cpp final_project::TurtleController -p turtle_name:=abc
```

**2. Verificar componentes carregados:**
```bash
ros2 component list
```

Saída esperada:
```
/ComponentManager
   1  /turtle_controller
```

**3. Verificar lifecycle nodes disponíveis:**
```bash
ros2 lifecycle nodes
```

Saída esperada:
```
/turtle_controller
```

**4. Verificar estado atual do lifecycle:**
```bash
ros2 lifecycle get /turtle_controller
```

Saída esperada:
```
unconfigured [1]
```

**5. Configurar o node (spawna a turtle):**
```bash
ros2 lifecycle set /turtle_controller configure
```

**6. Ativar o node (action server fica pronto):**
```bash
ros2 lifecycle set /turtle_controller activate
```

**7. Verificar action servers disponíveis:**
```bash
ros2 action list
```

Saída esperada:
```
/move_turtle_abc
```

**8. Enviar um goal para mover a turtle:**
```bash
ros2 action send_goal /move_turtle_abc my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, angular_vel_z: 0.5, duration_sec: 3}"
```

**9. Testar desativação durante execução (aborts goal):**
```bash
# Enviar goal longo:
ros2 action send_goal /move_turtle_abc my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, angular_vel_z: 1.0, duration_sec: 10}" --feedback

# Em outro terminal, desativar durante execução:
ros2 lifecycle set /turtle_controller deactivate
```

**10. Limpar o node (mata a turtle):**
```bash
ros2 lifecycle set /turtle_controller cleanup
```

**11. Descarregar o componente:**
```bash
ros2 component unload /ComponentManager 1
```

## Carregar Múltiplos Componentes (Mesmo Container)

Você pode carregar vários componentes no mesmo container para controlar múltiplas turtles:

**1. Carregar primeiro componente (abc):**
```bash
ros2 component load /ComponentManager final_project_level3_cpp final_project::TurtleController -p turtle_name:=abc
```

**2. Carregar segundo componente (def) - remapear node_name:**
```bash
ros2 component load /ComponentManager final_project_level3_cpp final_project::TurtleController -r __node:=turtle_controller2 -p turtle_name:=def
```

Ou usando `--node-name`:
```bash
ros2 component load /ComponentManager final_project_level3_cpp final_project::TurtleController --node-name turtle_controller2 -p turtle_name:=def
```

**3. Verificar componentes carregados:**
```bash
ros2 component list
```

Saída esperada:
```
/ComponentManager
   1  /turtle_controller
   2  /turtle_controller2
```

**4. Verificar lifecycle nodes:**
```bash
ros2 lifecycle nodes
```

Saída esperada:
```
/turtle_controller
/turtle_controller2
```

**5. Configurar e ativar ambos:**
```bash
# Primeiro componente
ros2 lifecycle set /turtle_controller configure
ros2 lifecycle set /turtle_controller activate

# Segundo componente
ros2 lifecycle set /turtle_controller2 configure
ros2 lifecycle set /turtle_controller2 activate
```

**6. Verificar action servers (agora temos 2):**
```bash
ros2 action list
```

Saída esperada:
```
/move_turtle_abc
/move_turtle_def
```

**7. Controlar ambas as turtles simultaneamente:**
```bash
# Terminal 1: Controlar turtle abc
ros2 action send_goal /move_turtle_abc my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, angular_vel_z: 0.5, duration_sec: 5}"

# Terminal 2: Controlar turtle def
ros2 action send_goal /move_turtle_def my_robot_interfaces/action/MoveTurtle "{linear_vel_x: -0.5, angular_vel_z: -0.8, duration_sec: 5}"
```

**8. Descarregar componentes:**
```bash
ros2 component unload /ComponentManager 1  # turtle_controller
ros2 component unload /ComponentManager 2  # turtle_controller2
```

## Execução Direta (Sem Container)

Também é possível executar diretamente como um executável standalone:

```bash
ros2 run final_project_level3_cpp turtle_controller_cpp --ros-args -p turtle_name:=turtle2
```

Depois seguir os mesmos passos de lifecycle:
```bash
ros2 lifecycle set /turtle_controller configure
ros2 lifecycle set /turtle_controller activate
ros2 action send_goal /move_turtle_turtle2 my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, angular_vel_z: 0.5, duration_sec: 3}"
```

## Diagrama de Estados do Lifecycle

```
unconfigured
     |
     | configure
     v
  inactive
     |
     | activate
     v
   active  <---- Aqui o action server aceita goals
     |
     | deactivate
     v
  inactive
     |
     | cleanup
     v
unconfigured
```

## Recursos

- **Component**: Pode ser carregado dinamicamente em um container
- **Lifecycle**: Controle explícito sobre inicialização e desligamento
- **Action Server**: MoveTurtle com validação e suporte a cancelamento
- **Race Condition Free**: Usa mutex para proteger goal_handle_
- **Deactivation Handling**: Aborta goals automaticamente quando desativado

## Validações do Goal

O action server rejeita goals que:
- `abs(linear_vel_x) > 3.0`
- `abs(angular_vel_z) > 2.0`
- `duration_sec <= 0`
- Servidor não estiver ativado (lifecycle state != active)
- Já existir um goal ativo
