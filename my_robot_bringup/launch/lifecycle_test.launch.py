from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()              # criar a descrição do launch

    number_node_name = "my_number_publisher"

    # criar o nó number_publisher como um lifecycle node
    number_node = LifecycleNode( 
        package="lifecycle_py",
        executable="number_publisher",
        name=number_node_name,
        namespace="",
    )
    
    # criar o nó lifecycle_node_manager, passando o nome do nó gerenciado como parâmetro
    lifecycle_node_manager = Node(
        package="lifecycle_py",
        executable="lifecycle_node_manager",
        parameters=[{"managed_node_name": number_node_name}],
    )

    ld.add_action(number_node)            # adicionar o nó number_publisher
    ld.add_action(lifecycle_node_manager) # adicionar o nó lifecycle_node_manager

    return ld