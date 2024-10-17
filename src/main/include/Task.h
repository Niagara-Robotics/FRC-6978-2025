#include <chrono>
#include "ctre/phoenix6/TalonFX.hpp"
#pragma once

class Task
{
protected:
    std::chrono::time_point<std::chrono::steady_clock> next_execution;

public:
    std::chrono::time_point<std::chrono::steady_clock> get_next_execution();
    virtual void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) = 0;
    virtual void call(bool robot_enabled, bool autonomous) = 0;
    virtual bool is_paused() = 0;
};

