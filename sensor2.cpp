#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

// Definindo os pinos de S1 a S5
const int sensorPins[] = {0, 2, 3, 4, 1};  // Ajuste para os pinos GPIO corretos
const int numSensors = 5;

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

int main() {
    setupPins();

    while (true) {
        std::cout << "Lendo valores dos sensores:" << std::endl;
        std::vector<int> values = readSensors();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Pausa para evitar leituras rápidas demais
    }

    return 0;
}
