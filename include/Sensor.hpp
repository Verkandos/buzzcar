#pragma once

class Sensor {
    protected:
        float value;
    public:
        virtual ~Sensor() = default;

        virtual void initialize(int pin) = 0;
        virtual float readValue() = 0;
};