#pragma once
#include <list>
#include <chrono>
#include <thread>
#include "Task.h"
#include <string>

class Scheduler
{
public:
    Scheduler(std::string name);
    void execute();
    void register_task(Task *task);
    void set_robot_status(bool enabled, bool autonomous);
    ~Scheduler();
private:
    std::list<Task *> tasks;
    std::thread *thread;
    std::string name;
    void log(const char* message);
    bool stop_flag = false;
    bool enabled = false;
    bool autonomous = false;
};

