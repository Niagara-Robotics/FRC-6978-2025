#pragma once
#include <chrono>

namespace controlchannel
{

template <class T> class ControlHandle;

template <class T> class ControlChannel
{
public: 
    using callback_t = void(*)(T);
private:
    T value;
    std::chrono::time_point<std::chrono::steady_clock> last_set_time;

    int controller_id = -1; // actual ids given out start at 1, -1 means nobody, 0 means channel owner
    int id_counter = 0; 
    bool permissive = true;
    callback_t callback;

public:
    ControlChannel(T value);
    ControlHandle<T> get_handle();

    bool take_control(int id, bool permissive);

    bool has_control(int id) {
        return id == this->controller_id;
    }
    
    bool set(int id, T value) {
        if(!has_control(id)) return false;
        this->value = value;
        if (callback != nullptr) callback(value);
        last_set_time = std::chrono::steady_clock::now();
        return true;
    };

    void release(int id) {
        if(!has_control(id)) return;
        this->controller_id = -1;
        this->permissive = true;
    }

    T get() {
        return value;
    }

    std::chrono::time_point<std::chrono::steady_clock> get_last_set_time() {
        return last_set_time;
    }

    void set_callback(callback_t callback) {
        this->callback = callback;
    };

    ~ControlChannel();
};

template <class T> class ControlHandle
{
private:
    T value;
    ControlChannel<T>* parent;
    int id;
public:
    ControlHandle(int id, ControlChannel<T>* parent) : id(id), parent(parent){};
    bool set(T value) {
        return parent->set(id, value);
    };

    T get() {
        return parent->get();
    };

    void release() {
        parent->release(id);
    }

    bool has_control() {
        return parent->has_control(id);
    }
    
    bool try_take_control() {
        return parent->take_control(id, true);
    };
    ~ControlHandle() {};
};

template <class T> ControlChannel<T>::ControlChannel(T init_value) : value(init_value)
{

}

template <class T> bool ControlChannel<T>::take_control(int id, bool permissivie) {
    if(controller_id == id) {
        this->permissive = permissive;
        return true;
    }

    if(!this->permissive && id != 0) return false;

    this->controller_id = id;
    this->permissive = permissive;
    return true;
}

template <class T> ControlHandle<T> ControlChannel<T>::get_handle() {
    id_counter++;
    return ControlHandle<T>(id_counter, this);
}

template <class T> ControlChannel<T>::~ControlChannel()
{
}

} // namespace controlchannel