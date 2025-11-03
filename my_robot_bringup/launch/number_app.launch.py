from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description(): 
    ld = LaunchDescription() 

    # caminho para o arquivo de configuração de parâmetros
    param_config = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "number_app.yaml") 


    number_publisher = Node(
        package="my_py_pkg",
        executable="number_publisher",
        namespace="/abc",
        name="my_number_publisher",
        remappings=[("/number", "/my_number")],
        # parameters=[{"number": 1, "timer_period": 3.5}] # definir parâmetros diretamente no launch file
        parameters=[param_config] # carregar parâmetros do arquivo de configuração

    )

    number_counter = Node(
        package="my_py_pkg",
        executable="number_counter",
        name="my_number_counter",
        remappings=[("/number", "/my_number")]
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)

    return ld