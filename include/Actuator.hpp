#pragma once
#include <string>

class Actuator {
    private:
        std::string actuatorType;

    public:
        virtual void activate() = 0;
};