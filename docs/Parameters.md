# ROS2 Parameters: Parâmetros Dinâmicos para seus Nós

## 1. Declarar parâmetros no código
No seu node Python:
```python
self.declare_parameter("number", 1)
self.declare_parameter("timer_period", 1.0)
```

## 2. Ver lista de parâmetros ativos
```bash
ros2 param list
```

## 3. Personalizar parâmetros pelo terminal
Ao rodar um node, defina parâmetros com `--ros-args -p`:
```bash
ros2 run my_py_pkg number_publisher --ros-args -p number:=3 -p timer_period:=3.0
ros2 run my_py_pkg robot_news_station --ros-args -p robot_name:=dani
```

## 4. Ver valor de um parâmetro
```bash
ros2 param get /nome_do_node nome_do_parametro
# Exemplo:
ros2 param get /turtlesim background_g
```

## 5. Usar arquivos YAML para parâmetros
1. Crie uma pasta para os arquivos YAML:
   ```bash
   mkdir yaml_params
   cd yaml_params
   ```
2. Crie um arquivo, exemplo `number_params.yaml`:
   ```yaml
   /number_publisher:
     ros__parameters:
       number: 5
       timer_period: 1.0
   ```
3. Rode o node usando o arquivo YAML:
   ```bash
   ros2 run my_py_pkg number_publisher --ros-args --params-file ~/yaml_params/number_params.yaml
   ```
   Se mudar o nome do node:
   ```bash
   ros2 run my_py_pkg number_publisher --ros-args -r __node:=number_publisher1 --params-file ~/yaml_params/number_params.yaml
   ```

## 6. Mudar valor do parâmetro em tempo de execução (callback)
No código do node:
```python
self.add_on_set_parameters_callback(self.parameters_callback)

def parameters_callback(self, params: list[Parameter]):
    for param in params:
        if param.name == "number":
            self.number_ = param.value
            self.get_logger().info(f"Parameter 'number' changed to: {self.number_}")
```

## 7. Obter e setar parâmetros em tempo real
- Obter valor atual:
  ```bash
  ros2 param get /number_publisher number
  ```
- Alterar valor:
  ```bash
  ros2 param set /number_publisher number 4
  ```

## 8. Ver efeito em tempo real
Use:
```bash
ros2 topic echo /number
```
para ver as mensagens refletindo a mudança do parâmetro.

---

Com parâmetros, você pode tornar seus nós ROS2 muito mais flexíveis e configuráveis, tanto na inicialização quanto em tempo de execução!
