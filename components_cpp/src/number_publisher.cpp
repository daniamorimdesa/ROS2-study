#include "rclcpp/rclcpp.hpp"
#include "components_cpp/number_publisher.hpp"

using namespace std::chrono_literals;

/*
Como construir um componente em C++:
1- incluir 'const rclcpp::NodeOptions & options' no construtor da classe que herda de rclcpp::Node

2 - adicionar duas linhas no fim do arquivo:
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::NumberPublisher)

3 - colocar a classe dentro de um namespace para evitar conflitos de nomes

4- criar o number_publisher.hpp com a implementação do construtor e dos métodos

5- adicionar a dependência 'rclcpp_components' no package.xml
6- adicionar o componente no arquivo CMakeLists.txt:
find_package(rclcpp_components REQUIRED)
ament_target_dependencies(components_cpp rclcpp rclcpp_components example_interfaces)
rclcpp_components_register_node(components_cpp PLUGIN "my_namespace::NumberPublisher" EXECUTABLE number_publisher_component)
7- construir o workspace com colcon build
8- ros2 component types : verificar se o componente está registrado
*/

namespace my_namespace {

NumberPublisher::NumberPublisher(const rclcpp::NodeOptions & options) : Node("number_publisher", options)
{
    number_ = 2;

    number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    number_timer_ = this->create_wall_timer(1000ms,
                                            std::bind(&NumberPublisher::publishNumber, this));
    RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
}


void NumberPublisher::publishNumber()
{
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
}

} // namespace my_namespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::NumberPublisher)