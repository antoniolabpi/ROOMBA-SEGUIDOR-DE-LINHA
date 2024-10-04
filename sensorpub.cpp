#include <chrono>
#include <memory>
#include <iostream>
#include <wiringPi.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher()
  : Node("sensor_publisher")
  {
    // Inicializa o WiringPi
    if (wiringPiSetup() == -1) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao iniciar o WiringPi.");
      throw std::runtime_error("Erro ao iniciar o WiringPi.");
    }

    // Configura os pinos dos sensores como entrada
    for (int i = 0; i < numSensors; i++) {
      pinMode(sensorPins[i], INPUT);
    }

    // Criando o publisher para publicar um array de valores float
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sensor_topic", 10);

    // Criando um timer que chama a função "timer_callback" a cada 1 segundo
    timer_ = this->create_wall_timer(1000ms, std::bind(&SensorPublisher::timer_callback, this));
  }

private:
  // Número de sensores e os pinos GPIO associados
  static const int numSensors = 5;
  int sensorPins[numSensors] = {11, 13, 15, 16, 18};

  // Leitura dos dados dos sensores
  std::vector<float> read_sensor_data() {
    std::vector<float> sensorValues;

    // Lê os valores de cada sensor e armazena no vetor
    for (int i = 0; i < numSensors; i++) {
        int sensorValue = digitalRead(sensorPins[i]);
        RCLCPP_INFO(this->get_logger(), "Sensor %d raw value: %d", i + 1, sensorValue);
        sensorValues.push_back(static_cast<float>(sensorValue));
}


    return sensorValues;
  }

  void timer_callback()
  {
    // Criando a mensagem que será publicada
    auto message = std_msgs::msg::Float32MultiArray();

    // Lendo os dados dos sensores
    std::vector<float> sensor_values = read_sensor_data();

    // Preenchendo a mensagem com os valores dos sensores
    message.data = sensor_values;

    // Publicando a mensagem no tópico
    RCLCPP_INFO(this->get_logger(), "Publishing sensor values:");
    for (size_t i = 0; i < sensor_values.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "Sensor %zu value: %.2f", i + 1, sensor_values[i]);
    }

    publisher_->publish(message);  // Publica os valores dos sensores
  }

  // Declarando o publisher
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

  // Declarando o timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Inicializando o ROS 2
  rclcpp::init(argc, argv);

  try {
    // Criando o nó do publisher para os dados do sensor
    rclcpp::spin(std::make_shared<SensorPublisher>());
  } catch (const std::exception & e) {
    std::cerr << "Erro: " << e.what() << std::endl;
    return -1;
  }

  // Encerrando o ROS 2
  rclcpp::shutdown();
  return 0;
}
