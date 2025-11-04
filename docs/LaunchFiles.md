# ROS2 Launch Files: Organização e Execução de Múltiplos Nós

## 1. Organização: Crie um pacote de bringup
Para manter seu workspace organizado, crie um pacote dedicado para arquivos de lançamento (launch):
```bash
ros2 pkg create my_robot_bringup
```

Remova as pastas `include` e `src` se não for usar código fonte:
```bash
rm -rf my_robot_bringup/include my_robot_bringup/src
```

## 2. Crie a pasta de launch
```bash
mkdir my_robot_bringup/launch
```

## 3. Edite o CMakeLists.txt para instalar os arquivos de launch
Adicione:
```cmake
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
```
E garanta que o arquivo tenha:
```cmake
find_package(ament_cmake REQUIRED)
ament_package()
```

## 4. Exemplo de launch file XML
Crie um arquivo, ex: `number_app.launch.xml`:
```xml
<launch>
   <node pkg="my_py_pkg" exec="number_publisher" />
   <node pkg="my_py_pkg" exec="number_counter" />
</launch>
```

Para dar nome aos nós:
```xml
<launch>
   <node pkg="my_py_pkg" exec="number_publisher" name="my_number_publisher" />
   <node pkg="my_py_pkg" exec="number_counter" name="my_number_counter" />
</launch>
```

## 5. Adicione dependências no package.xml
Inclua as dependências dos pacotes que você vai lançar:
```xml
<exec_depend>my_py_pkg</exec_depend>
```

## 6. Build e source
```bash
colcon build
source install/setup.bash
```

## 7. Executar um launch file (exemplo)
```bash
ros2 launch my_robot_bringup number_app.launch.xml
```

## 8. Launch files em Python
Você pode criar arquivos `.launch.py` para lógica mais avançada. Exemplo:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_py_pkg',
            executable='number_publisher',
            name='my_number_publisher'
        ),
        Node(
            package='my_py_pkg',
            executable='number_counter',
            name='my_number_counter'
        )
    ])
```
Execute com:
```bash
ros2 launch my_robot_bringup number_app.launch.py
```

## 9. Incluir um launch Python em um XML
```xml
<launch>
   <include file="$(find-pkg-share my_robot_bringup)/launch/number_app.launch.py" />
</launch>
```

## 10. Parâmetros, remap e namespace
Você pode passar parâmetros, remapear tópicos e definir namespaces diretamente no XML ou Python. Veja exemplos nos arquivos do seu repositório.

## 11. Configuração de parâmetros
Crie uma pasta `config/` para arquivos YAML de parâmetros:
```bash
mkdir my_robot_bringup/config
```

---

Com launch files, você pode iniciar múltiplos nós, definir parâmetros, remapear tópicos e organizar sua aplicação ROS2 de forma profissional!
