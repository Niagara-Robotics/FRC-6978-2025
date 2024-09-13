#include "Task.h"

std::chrono::time_point<std::chrono::steady_clock> Task::get_next_execution()
{
    return this->next_execution;
}