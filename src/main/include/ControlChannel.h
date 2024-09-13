#pragma once

template <class T> class ControlHandle;

template <class T> class ControlChannel
{
private:
    T value;
    int controller_id = 0;
    bool permissive = true;
public:
    ControlChannel(T value);
    bool take_control(int id, bool permissive);
    ControlHandle<T> get_handle();
    ~ControlChannel();
};

template <class T> class ControlHandle
{
private:
    T value;
    ControlChannel<T>* parent;
    int id;
public:
    ControlHandle(int id, ControlChannel<T>* parent) : id(id){};
    bool set(T value) {

    };
    ~ControlHandle();
};



template <class T> ControlChannel<T>::ControlChannel(T init_value) : value(init_value)
{

}

template <class T> ControlChannel<T>::~ControlChannel()
{
}
