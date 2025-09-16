#include <Arduino.h>
#include "ControlSubsystem.hpp"

// Global pointer to control subsystem
ControlSubsystem* buzzcar = nullptr;


void setup() {
  // put your setup code here, to run once:

  // Frequency of the CPU: Setting to 160MHz
  // Note: This setting is specific to certain boards like ESP32
  setCpuFrequencyMhz(160);

  // Initialize serial monitor for debugging
  Serial.begin(9600);

  // Check if CPU frequency is set correctly
  Serial.print("CPU Frequency set to: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  // Initialize Control Subsystem
  buzzcar = new ControlSubsystem();
  buzzcar->initialize();
  Serial.println("Control Subsystem Initialized");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (buzzcar != nullptr) {
    buzzcar->update();
  }
}

// put function definitions here:
