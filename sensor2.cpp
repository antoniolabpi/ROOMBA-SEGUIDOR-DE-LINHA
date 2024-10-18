#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#define Kp 0.018
#define Kd 0.0001
#define KI 0.00002

// Definindo os pinos de S1 a S5
const int sensorPins[] = {0, 2, 3, 4, 1};  // Ajuste para os pinos GPIO corretos
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
    int position = 0; // Calcule a posição com base nos valores dos sensores

    // Exemplo de cálculo da posição (você deve ajustar conforme sua necessidade)
    for (int i = 0; i < numSensors; i++) {
        position += sensorValues[i] * (i + 1); // Exemplo: ponderação pela posição do sensor
    }

    int error = 3500 - position; // Ajuste o valor de referência conforme necessário
    double P = error;
    I += error; // Acumula o erro para a parte integral
    double D = error - lasterror; // Derivada do erro
    lasterror = error;

    double variavel = (P * Kp) + (I * KI) + (D * Kd); // Cálculo do controle PID

    // Lógica de controle do motor (exemplo)
    if (position > 3800) {
        // Adicione lógica para mover o robô para a esquerda
    } else if (position < 2900) {
        // Adicione lógica para mover o robô para a direita
    } else {
        // Mantenha o robô em linha
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
