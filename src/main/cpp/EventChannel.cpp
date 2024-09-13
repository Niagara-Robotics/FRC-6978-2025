#include "EventChannel.h"

EventChannel::EventChannel(void *callback) {
    callback = callback;
}

void EventChannel::add_incoming(int id) {
    incoming_mutex.lock();
    incoming_queue.push_back(id);
    incoming_mutex.unlock();
}

void EventChannel::add_outgoing(int id) {
    outgoing_queue.push_back(id);
}

int EventChannel::pop_outgoing() {
    outgoing_queue.pop_front();
}

void EventChannel::execute() {
    while (true)
    {
        incoming_mutex.lock();
        if (incoming_queue.size() < 1) {
            incoming_mutex.unlock();
            continue;
        }
        int id = incoming_queue.front();
        incoming_queue.pop_front();
        incoming_mutex.unlock();

        callback(id);
    }
    
}

EventChannel::~EventChannel() {
    executor_thread->join();
    executor_thread->~thread();
}