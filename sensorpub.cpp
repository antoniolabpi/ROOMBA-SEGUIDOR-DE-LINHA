#include <chrono>
#include <vector>
#include <thread>
#include <memory>
#include <iostream>
#include <wiringPi.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

#define Kp 0.018
#define Kd 0.0001

// Definindo os pinos de S1 a S5
uint16_t sensorPins[] = {0, 2, 3, 4, 1};  // Ajuste para os pinos GPIO corretos
const int numSensors = 5;
int lasterror = 0;
double I = 0;  // Inicializa a variável I

class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher() : Node("sensor_publisher")
    {
        // Inicializa o WiringPi
        if (wiringPiSetup() == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Erro ao iniciar o WiringPi.");
            throw std::runtime_error("Erro ao iniciar o WiringPi.");
        }

        // Configura os pinos dos sensores como entrada
        for (int i = 0; i < numSensors; i++)
        {
            pinMode(sensorPins[i], INPUT);
        }

        // Criando o publisher para publicar no tópico cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Criando um timer que chama a função "timer_callback" a cada 500ms
        timer_ = this->create_wall_timer(500ms, std::bind(&SensorPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Lógica de controle PID e leitura dos sensores
        PID();
    }

    std::vector<int> readSensors()
    {
        std::vector<int> sensorValues;
        for (int i = 0; i < numSensors; i++)
        {
            int value = digitalRead(sensorPins[i]);
            sensorValues.push_back(value);
            RCLCPP_INFO(this->get_logger(), "Sensor %d valor: %d", i + 1, value);
        }
        return sensorValues;
    }

    void PID()
    {
        // Leitura dos sensores
        std::vector<int> sensorValues = readSensors();
        uint16_t position = 0; // Calcule a posição com base nos valores dos sensores

        // Exemplo de cálculo da posição
        for (int i = 0; i < 5; i++)
        {
            position += sensorValues[i] * (i + 1); // Exemplo: ponderação pela posição do sensor
        }

        int error = 3500 - position; // Ajuste o valor de referência conforme necessário
        double P = error;
        I += error;               // Acumula o erro para a parte integral
        double D = error - lasterror; // Derivada do erro
        lasterror = error;

        double variavel = (P * Kp) + (D * Kd); // Cálculo do controle PID
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.5;  // Velocidade linear de 0.5 m/s
        cmd_vel_msg.angular.z = 0.0; // Velocidade angular

        // Lógica de controle do motor
        if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0)
        {
            cmd_vel_msg.linear.x = 0.5;  // Mover para frente
            cmd_vel_msg.angular.z = 0.0;
        }
        else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0)
        {
            cmd_vel_msg.angular.z = variavel * 0.01; // Mover para a esquerda
        }
        else if (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0)
        {
            cmd_vel_msg.angular.z = variavel * 0.01; // Mover mais para a esquerda
        }
        else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 0)
        {
            cmd_vel_msg.angular.z = -(variavel * 0.01); // Mover para a direita
        }
        else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1)
        {
            cmd_vel_msg.angular.z = -(variavel * 0.01); // Mover mais para a direita
        }
        else
        {
            cmd_vel_msg.linear.x = 0.5;  // Mover para frente se todos os sensores forem 0
            cmd_vel_msg.angular.z = 0.0;
        }

        // Publica o comando de velocidade no tópico cmd_vel
        publisher_->publish(cmd_vel_msg);
    }

    // Declarando o publisher e o timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // Inicializando o ROS 2
    rclcpp::init(argc, argv);

    try
    {
        // Criando o nó do publisher para os dados do sensor
        rclcpp::spin(std::make_shared<SensorPublisher>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Erro: " << e.what() << std::endl;
        return -1;
    }

    // Encerrando o ROS 2
    rclcpp::shutdown();
    return 0;
}
