#pragma

class PIDController {
    private:
        float kp; // Proportional gain
        float ki; // Integral gain
        float kd; // Derivative gain

        // PID state variables
        float previousError; // Last error value for derivate calculation
        float integral; // Running sum of errors for integral term
        unsigned long previousTime; // Last computation time time

        // Output constraints
        float outputMin; // Minimum output valie
        float outputMax; // Maximum output value

        // Configuration
        float integralLimit; // Maximum integral value (prevents windup)

    public:
        // Constructor
        PIDController(float kp, float ki, float kd);

        // Compute PID output based on setpoint and current input
        float compute(float setpoint, float input);
        
        // Set output limits to constrain the PID output
        void setOutputLimits(float min, float max);

        // Update PID tuning parameters
        void setTunings(float kp, float ki, float kd);

        // Reset PID controller state 
        // Useful when starting line following or changing states (turning stop, etc)
        void reset();

        // Set integral windup limit
        void setIntegralLimit(float limit);

        // Get current PID parameters 
        void getTunings(float& kp, float& ki, float& kd) const;

};