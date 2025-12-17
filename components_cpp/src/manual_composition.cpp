#include "rclcpp/rclcpp.hpp"
#include "components_cpp/node1.hpp"
#include "components_cpp/node2.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Criar um executor single-threaded
    rclcpp::executors::SingleThreadedExecutor executor;

    // Criar instâncias dos nodes
    auto node1 = std::make_shared<Node1>();
    auto node2 = std::make_shared<Node2>();

    // Adicionar os nodes ao executor
    executor.add_node(node1);
    executor.add_node(node2);

    // Executar o executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
