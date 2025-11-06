#pragma once
#include <memory>
#include <vector>
#include <Motor.hpp>
#include "Event.hpp"
#include "LineDetector.hpp"
#include "PIDController.hpp"
// DO NOT ADD "FSM.hpp" its creating circular dependencies


// Forward Declarations
class FSM;
class State;
class Motor;
class LineDetector;
class PIDController;
class Event;

class ControlSubsystem {
    public:
        // Turn memory for recovery from line loss
        enum class LastTurn {
            NONE,
            LEFT,
            RIGHT
        };
        
    private:
        std::unique_ptr<FSM> fsm;

        // State pointers - these are managed by FSM
        State* idleState;
        State* forwardState;
        State* turnLeftState;
        State* turnRightState;
        State* stopState;

        // Hardware components
        Motor* motorA;
        Motor* motorB;
        PhotoSensor* sensorLeft;
        PhotoSensor* sensorCenter;
        PhotoSensor* sensorRight;
        LineDetector* lineDetector;
        PIDController* pidController;

        // Turn memory tracking
        LastTurn lastTurnDirection;
        unsigned long lastLineSeenTime;
        static const unsigned long LINE_LOST_TIMEOUT = 3000; // 3 seconds

        // Helper method to generate events based on sensor readings
        Event generateEvent();
    
    public:
        ControlSubsystem();
        ~ControlSubsystem();

        // Core functionality
        void initialize();
        void update();
        Event handleEvent();

        // Getters for hardware components
        Motor* getMotorA() const { return motorA; }
        Motor* getMotorB() const { return motorB; }
        LineDetector* getLineDetector() const { return lineDetector; }
        PIDController* getPIDController() const { return pidController; }
        FSM* getFSM() { return fsm.get(); }

        // Turn memory getters/setters
        LastTurn getLastTurn() const { return lastTurnDirection; }
        void setLastTurn(LastTurn turn) { lastTurnDirection = turn; }
};