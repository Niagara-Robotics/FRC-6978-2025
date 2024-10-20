#pragma once

#include <vector>
#include <chrono>

#include "Faults.h"

#define FAULTMANAGER_WATCHDOG_TIMEOUT 4 //seconds

class Fault {
public:
    bool affect_subsystem;
    std::chrono::time_point<std::chrono::steady_clock> expiry_time;
    bool expires = false;
    FaultIdentifier fault;

    Fault(bool affect_subsystem, FaultIdentifier fault);
};

class FaultManager {

private:
    std::vector<Fault> faults;
    std::chrono::time_point<std::chrono::steady_clock> last_watchdog_checkin;
    bool subsystem_status = true;
    std::string subsystem_name;
public:
    FaultManager(std::string name): subsystem_name(name) {};

    void add_fault(Fault fault);
    void clear_fault(Fault fault);
    void feed_watchdog();

    bool get_status();
    bool check_watchdog();

    std::string get_name();

    ~FaultManager() {};

};

class GlobalFaultManager {
private:
    bool global_status = true;

public:
    std::vector<FaultManager*> managers;

    void push_data();
    void register_manager(FaultManager* manager);
    void refresh();
};