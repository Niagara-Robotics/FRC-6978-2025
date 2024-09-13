#include "Scheduler.h"
#include <iostream>

Scheduler::Scheduler(std::string name)
{
    this->name = name;
    this->thread = new std::thread(&Scheduler::execute, this);
}

void Scheduler::register_task(Task *task) 
{
    this->tasks.push_front(task);
    task->schedule_next(std::chrono::steady_clock::now());
    log("Registered task");
}

void Scheduler::log(const char* message) {
    std::cout << "[scheduler:" << this->name << "] " << message << "\n";
}

void Scheduler::execute() 
{
    log("Hello!");
    while(true) {
        for (Task *task : tasks)
        {
            if(stop_flag) {
                return;
            }
            //compensates for slight delay in the scheduling system
            if ((std::chrono::steady_clock::now() - std::chrono::microseconds(3)) > task->get_next_execution())
            {
                task->schedule_next(std::chrono::steady_clock::now());
                task->call();
            }
        }
        usleep(150);
    }
}

Scheduler::~Scheduler()
{
    stop_flag = true;
    this->thread->join();
    this->thread->~thread();
}