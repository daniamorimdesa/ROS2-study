# Introdução: Como Escrever e Rodar Programas com ROS2

## 1. Criar uma workspace do ROS2
Na sua home directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/
```

## 2. Criar um package Python
Entre na pasta `src` da workspace:
```bash
cd ~/ros2_ws/src/
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

## 3. Criar um package C++ (opcional)
```bash
cd ~/ros2_ws/src/
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

## 4. Abrir o VS Code a partir da pasta src
```bash
cd ~/ros2_ws/src/
code .
```

## 5. Sempre buildar a partir do workspace
Volte para a raiz da workspace:
```bash
cd ~/ros2_ws/
```

## 6. Buildar todos os pacotes
```bash
colcon build
```

## 7. Buildar apenas um package específico
```bash
colcon build --packages-select my_py_pkg
```

## 8. Escrever um node dentro do package
- Crie um arquivo Python dentro de `my_py_pkg/my_py_pkg/`.
- Configure o `setup.py` para incluir o novo node.
- Torne o arquivo executável: `chmod +x nome_do_node.py`

## 9. Compilar e rodar
- Compile com `colcon build`.
- Carregue o ambiente:
  ```bash
  source install/setup.bash
  ```
- Rode o node:
  ```bash
  ros2 run my_py_pkg nome_do_executavel
  ```

## 10. Rodar todos os nodes de um package (usando launch files)
```bash
ros2 launch <nome_do_package> <nome_do_launch_file>
```

---

Com esses passos, você pode criar, compilar e rodar programas em ROS2, tanto em Python quanto em C++, de forma organizada e eficiente!
