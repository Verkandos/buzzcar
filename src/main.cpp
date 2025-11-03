#include <Arduino.h>
#include "ControlSubsystem.hpp"
#include "UserInterface.hpp"
#include "ControlConfig.hpp"
#include "Speaker.h"
#include "Screen.h"
#include "FSM.hpp"

// Global pointer to control subsystem
ControlSubsystem* buzzcar;
UserInterface* ui;

void setup() {
  Serial.begin(115200);
  // Wait for serial connection
  while (!Serial && millis() < 2000) {
      delay(100);
  }
  delay(1000);

  // Get config and turn parameters
  ControlConfig& config = ControlConfig::getInstance();

  config.printConfig();
  
  Serial.println("BuzzCar Control System Starting...");

  pinMode(config.pins.protectionPin, INPUT);
  Serial.println("Pin 23 set to high impedance (INPUT mode)");


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
    
    if (ui->isSystemOn()) {
      Serial.println("System turned ON");
    } else {
      Serial.println("System turned OFF - Stopping motors");
      
      // Stop all motors immediately
      buzzcar->getMotorA()->stop();
      buzzcar->getMotorB()->stop();
      
      // Reset FSM to idle state
      buzzcar->getFSM()->reset();
      
      // Stop audio and show STOP on screen
      speakerStop();
      showDirection(0);
    }
  }

  // Update control system if enabled
  if (ui->isSystemOn()) {
    buzzcar->update();
  }

  delay(config.timing.mainLoopDelay); // Loop delay
  
}

