#pragma once
#include <Arduino.h>
/**
 * @brief Centralized configuration for control subsystem parameters
 * 
 * This class provides a single location for all tunable parameters including
 * motor speeds, PID controller gains, sensor thresholds, and timing values.
 * All parameters can be modified at runtime for easy tuning and testing.
 * 
 * @note Use singleton pattern to ensure consistent configuration
 * 
 */
class ControlConfig {
    private:
        ControlConfig() = default; // Private constructor for singleton

    public:
        // Singleton instance access
        static ControlConfig& getInstance() {
            static ControlConfig instance;
            return instance;
        }

        // === MOTOR CONFIGURATION ===
        struct MotorSettings {
            int baseSpeed = 50;      // Base speed for motors (0-100%)
            int turnSpeed = 40;      // Speed during turns (0-100%)
            int minStartPWM = 20; // Minimum PWM to start motor movement
            int maxPWM = 255;        // Maximum PWM value
            int motorFrequency = 10000; // PWM frequency in Hz (10kHz)
        } motor;

        // === PID CONTROLLER CONFIGURATION ===
        struct PIDSettings {
            float Kp = 2.0f;         // Proportional gain
            float Ki = 0.1f;         // Integral gain
            float Kd = 0.5f;         // Derivative gain
            float outputMin = -40.0f; // Minimum PID output
            float outputMax = 40.0f;  // Maximum PID output
            float integralLimit = 25.0f; // Integral windup limit
        } pid;

        // === LINE DETECTION CONFIGURATION ===
        struct LineSettings {
            int blackThreshold = 3000;    // ADC threshold for line detection (0-4095) LOOK AT FILES for;
            int whiteThreshold = 100; // ADC threshold for white line detection (0-4095)
            int sensorReadInterval = 10; // Sensor read interval in milliseconds
            int samplingCount = 8;      // Number of samples for averaging
        } sensors;

        // == TIMING CONFIGURATION ===
        struct TimingSettings {
            int mainLoopDelay = 50; // Control loop interval in milliseconds
            int turnDuration = 500;      // Timeout for states in milliseconds
            int stopTimeout = 2000;      // Timeout for stop state in milliseconds
        } timing;

        // === AUDIO/VISUAL CONFIGURATION ===
        struct FeedbackSettings {
            int audioVolume = 50; // Speaker volume (0-100%)
            int audioFrequency = 1000; // Speaker frequency in Hz
            bool enableAudio = true; // Enable audio feedback
            bool enableDisplay = true; // Enable visual display
        } feedback;

        // === SYSTEM CONFIGURATION ===
        struct SystemSettings {
            bool debugMode = true; // Enable debug logging
            bool enableButtonControl = true;  // Enable user button control
        } system;

        // === CONFIGURATION METHODS ===

        /**
         * @brief Prints all current configuration values to Serial
         * @note Useful for debugging and verification
         */
        void printConfig() const {
            Serial.println("=== Control Configuration ===");
            Serial.println("-- Motor Settings --");
            Serial.print("Base Speed: "); Serial.println(motor.baseSpeed);
            Serial.print("Turn Speed: "); Serial.println(motor.turnSpeed);
            Serial.print("Min Start PWM: "); Serial.println(motor.minStartPWM);
            Serial.print("Max PWM: "); Serial.println(motor.maxPWM);
            Serial.print("Motor Frequency: "); Serial.println(motor.motorFrequency);
            Serial.println("-- PID Settings --");
            Serial.print("Kp: "); Serial.println(pid.Kp);
            Serial.print("Ki: "); Serial.println(pid.Ki);
            Serial.print("Kd: "); Serial.println(pid.Kd);
            Serial.print("Output Min: "); Serial.println(pid.outputMin);
            Serial.print("Output Max: "); Serial.println(pid.outputMax);
            Serial.print("Integral Limit: "); Serial.println(pid.integralLimit);
            Serial.println("-- Sensor Settings --");
            Serial.print("Black Threshold: "); Serial.println(sensors.blackThreshold);
            Serial.print("White Threshold: "); Serial.println(sensors.whiteThreshold);
            Serial.print("Sensor Read Interval: "); Serial.println(sensors.sensorReadInterval);
            Serial.println("-- Timing Settings --");
            Serial.print("Main Loop Delay: "); Serial.println(timing.mainLoopDelay);
            Serial.print("Turn Duration: "); Serial.println(timing.turnDuration);
            Serial.print("Stop Timeout: "); Serial.println(timing.stopTimeout);
            Serial.println("-- Feedback Settings --");
            Serial.print("Audio Volume: "); Serial.println(feedback.audioVolume);
            Serial.print("Audio Frequency: "); Serial.println(feedback.audioFrequency);
            Serial.print("Enable Audio: "); Serial.println(feedback.enableAudio);
            Serial.print("Enable Display: "); Serial.println(feedback.enableDisplay);
            Serial.println("-- System Settings --");
            Serial.print("Debug Mode: "); Serial.println(system.debugMode);
            Serial.print("Enable Button Control: "); Serial.println(system.enableButtonControl);
            Serial.println("============================");
        }
    };