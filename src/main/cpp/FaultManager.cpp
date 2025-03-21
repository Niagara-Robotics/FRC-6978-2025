#include "FaultManager.h"

#include <frc/smartdashboard/SmartDashboard.h>

void FaultManager::add_fault(Fault new_fault) {
    for(Fault old_fault : faults) {
        if(old_fault.fault == new_fault.fault) {
            old_fault.expiry_time = new_fault.expiry_time;
            return;
        }
    }
    faults.push_back(new_fault);
}

void FaultManager::clear_fault(Fault new_fault) {
    for(std::vector<Fault>::iterator it=faults.begin(); it !=faults.end(); it++) {
        if((*it).fault == new_fault.fault) {
            faults.erase(it);
            return;
        }
    }
}

bool FaultManager::get_fault(FaultIdentifier id, Fault *fault) {
    for(std::vector<Fault>::iterator it=faults.begin(); it != faults.end(); it++) {
        if((*it).fault == id) {
            if(fault != nullptr) *fault = (*it);
            return true;
        }
    }
    return false;
}

void FaultManager::feed_watchdog() {
    last_watchdog_checkin = std::chrono::steady_clock::now();
}

bool FaultManager::get_status() {
    subsystem_status = true;
    for(Fault fault : faults) {
        if(fault.affect_subsystem) {
            subsystem_status = false;
        }
    }
    return subsystem_status;
}

std::string FaultManager::get_faults() {
    std::string output = "";
    for(Fault fault : faults) {
        output.append(std::to_string((int)fault.fault) + ",");
    }
    return output;
}

bool FaultManager::check_watchdog() {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_watchdog_checkin) < std::chrono::seconds(FAULTMANAGER_WATCHDOG_TIMEOUT);
}

std::string FaultManager::get_name() {
    return subsystem_name;
}

void GlobalFaultManager::register_manager(FaultManager *manager) {
    managers.push_back(manager);
}

void GlobalFaultManager::refresh() {
    bool status = true;
    for(FaultManager *manager: managers) {
        if(!manager->get_status()) {
            status = false;
        }
        frc::SmartDashboard::PutBoolean("fm_" + manager->get_name() + "_status", manager->get_status());
        frc::SmartDashboard::PutBoolean("fm_" + manager->get_name() + "_watchdog", manager->check_watchdog());
        frc::SmartDashboard::PutString("fm_" + manager->get_name() + "_faults", manager->get_faults());
    }
    global_status = status;
}

void GlobalFaultManager::push_data() {

}