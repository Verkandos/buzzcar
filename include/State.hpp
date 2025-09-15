#pragma once
#include <string>

class State {
    private:
        std::string name;

    public:
        State(const std::string& stateName) : name(stateName) {}
        virtual ~State() = default;

        virtual void onEntry() = 0;
        virtual void onUpdate() = 0;
        virtual void onExit() = 0;

        std::string getName() const { return name; }
};