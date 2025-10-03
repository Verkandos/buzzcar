#pragma once
#include <string>
#include "ControlSubsystem.hpp"

class ControlSubsystem;

class State {
    private:
        std::string name;

    public:
        State(const std::string& stateName) : name(stateName) {}
        // Default Constructor
        State() : name("UnknownState") {}

        virtual ~State() = default;

        virtual void onEntry(ControlSubsystem* context) = 0;
        virtual void onUpdate(ControlSubsystem* context) = 0;
        virtual void onExit(ControlSubsystem* context) = 0;

        const char* getName() const { return name.c_str(); }
};