#include <Arduino.h>
#include "ControlSubsystem.hpp"
#include "UserInterface.hpp"

// Global pointer to control subsystem
ControlSubsystem* buzzcar;
UserInterface* ui;

void setup() {
  Serial.begin(115200);
  Serial.println("BuzzCar Control System Starting...");

  // Initialize Control Subsystem
  buzzcar = new ControlSubsystem();
  ui = new UserInterface(11); // User button pin

  buzzcar->initialize();
  ui->initialize();
  Serial.println("System ready! Press button to start.");
}

void loop() {
  // Update user interface
  if (ui->wasButtonPressed()) {
    ui->toggleSystem();
    Serial.print("System ON");
  }

  // Update control system if enabled
  if (ui->isSystemOn()) {
    buzzcar->update();
  }

  delay(50); // Loop delay
  
}

