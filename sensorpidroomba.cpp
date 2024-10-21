#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#define Kp 0.018
#define Kd 0.0001

// Definindo os pinos de S1 a S5
uint16_t sensorPins[] = {0, 2, 3, 4, 1};  // Ajuste para os pinos GPIO corretos
const int numSensors = 5;
int lasterror = 0;
int I = 0;
double I = 0;  // Inicializa a variável I

void setupPins() {
    wiringPiSetup();
    for (int i = 0; i < numSensors; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

std::vector<int> readSensors() {
    std::vector<int> sensorValues;
    for (int i = 0; i < numSensors; i++) {
        int value = digitalRead(sensorPins[i]);
        sensorValues.push_back(value);
        std::cout << "Sensor " << (i + 1) << " valor: " << value << std::endl;
    }
    return sensorValues;
}

void PID() {
    // Leitura dos sensores
    std::vector<int> sensorValues = readSensors();
    uint16_t position = 0; // Calcule a posição com base nos valores dos sensores

    // Exemplo de cálculo da posição (você deve ajustar conforme sua necessidade)
    for (int i = 0; i < 5; i++) {
        position += sensorValues[i] * (i + 1); // Exemplo: ponderação pela posição do sensor
    }

    int error = 3500 - position; // Ajuste o valor de referência conforme necessário
    double P = error;
    I += error; // Acumula o erro para a parte integral
    double D = error - lasterror; // Derivada do erro
    lasterror = error;

    double variavel = (P * Kp) + (D * Kd); // Cálculo do controle PID
    geometry_msgs__msg__Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.5;  // Velocidade linear de 0.5 m/s
    cmd_vel_msg.angular.z = 0.0; // Velocidade angular de 0.1 rad/s
    // Lógica de controle do motor
    if (sensorPins[0]==0 && sensorPins[1]==0 && sensorPins[2]==1 && sensorPins[3]==0 && sensorPins[4]==0) {
        // Adicione lógica para mover o robô para a frente
        cmd_vel_msg.linear.x = 0.5;  // Velocidade linear de 0.5 m/s
        cmd_vel_msg.angular.z = 0.0; // Velocidade angular de 0.0 rad/s
    }
    if (sensorPins[0]==0 && sensorPins[1]==1 && sensorPins[2]==0 && sensorPins[3]==0 && sensorPins[4]==0) {
        // Adicione lógica para mover o robô para a esquerda
        cmd_vel_msg.angular.z = variavel*0.01; // Velocidade angular positivo
    } 
    if (sensorPins[0]==1 && sensorPins[1]==0 && sensorPins[2]==0 && sensorPins[3]==0 && sensorPins[4]==0) {
        // Adicione lógica para mover o robô para a esquerda
        cmd_vel_msg.angular.z = variavel*0.01; // Velocidade angular positivo
    } 
    if (sensorPins[0]==0 && sensorPins[1]==0 && sensorPins[2]==0 && sensorPins[3]==1 && sensorPins[4]==0) {
        // Adicione lógica para mover o robô para a direita
        cmd_vel_msg.angular.z = -(variavel*0.01); // Velocidade angular negativo
    }
    if (sensorPins[0]==0 && sensorPins[1]==0 && sensorPins[2]==0 && sensorPins[3]==0 && sensorPins[4]==1) {
        // Adicione lógica para mover o robô para a direita 
        cmd_vel_msg.angular.z = -(variavel*0.01); // Velocidade angular negativo
    }
    else{
        //Adicione lógica para mover o robô para a frente
        cmd_vel_msg.linear.x = 0.5;  // Velocidade linear de 0.5 m/s
        cmd_vel_msg.angular.z = 0.0; // Velocidade angular de 0.0 rad/s
    }
}
int main() {
    setupPins();

    while (true) {
        std::cout << "Lendo valores dos sensores:" << std::endl;
        PID(); // Chama a função PID a cada iteração
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Pausa para evitar leituras rápidas demais
    }

    return 0;
}