#pragma once
#include <list>
#include <thread>
#include <mutex>

class EventChannel
{
private:
    void (*callback)(int id);
    std::list<int> outgoing_queue;
    std::list<int> incoming_queue;
    std::thread *executor_thread;
    std::mutex incoming_mutex;
    std::mutex outgoing_mutex;
    void execute();
public:
    EventChannel(void *callback);
    void add_incoming(int id);
    void add_outgoing(int id);
    int pop_outgoing();
    ~EventChannel();
};
