#pragma once
#include <string>
#include "ControlSubsystem.hpp"

class State {
    private:
        std::string name;

    public:
        State(const std::string& stateName) : name(stateName) {}
        virtual ~State() = default;

        virtual void onEntry(ControlSubsystem* context) = 0;
        virtual void onUpdate(ControlSubsystem* context) = 0;
        virtual void onExit(ControlSubsystem* context) = 0;

        std::string getName() const { return name; }
};