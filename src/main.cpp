#include <Arduino.h>
#include "ControlSubsystem.hpp"
#include "UserInterface.hpp"
#include "ControlConfig.hpp"

// Global pointer to control subsystem
ControlSubsystem* buzzcar;
UserInterface* ui;

void setup() {
  Serial.begin(115200);

  // Get config and turn parameters
  ControlConfig& config = ControlConfig::getInstance();

  config.printConfig();
  
  Serial.println("BuzzCar Control System Starting...");

  // Initialize Control Subsystem
  buzzcar = new ControlSubsystem();
  ui = new UserInterface(config.pins.userButton); // User button pin from config

  buzzcar->initialize();
  ui->initialize();
  Serial.println("System ready! Press button to start.");
}

void loop() {
  ControlConfig& config = ControlConfig::getInstance();
  // Update user interface
  if (ui->wasButtonPressed()) {
    ui->toggleSystem();
    Serial.print("System ON");
  }

  // Update control system if enabled
  if (ui->isSystemOn()) {
    buzzcar->update();
  }

  delay(config.timing.mainLoopDelay); // Loop delay
  
}

