#include "rclcpp/rclcpp.hpp" 

class MyNode : public rclcpp::Node // define a classe MyNode que herda de rclcpp::Node
{

public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello World"); // registra uma mensagem de log ao criar o nó
        timer_ = this->create_wall_timer(               // cria um timer que chama timerCallback a cada segundo
            std::chrono::seconds(1),                    // intervalo de 1 segundo
            std::bind(&MyNode::timerCallback, this)     // vincula o método timerCallback ao timer
        );
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_); // registra o valor do contador e incrementa
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp:: init(argc, argv); // inicializa o rclcpp com os argumentos fornecidos
    auto node = std::make_shared<MyNode>(); // cria uma instância compartilhada do nó MyNode
    rclcpp::spin(node); // mantém o nó ativo para processar callbacks (até que se aperte Ctrl+C)
    rclcpp:: shutdown(); // finaliza o rclcpp
    return 0; // retorna 0 para indicar que o programa terminou com sucesso
}