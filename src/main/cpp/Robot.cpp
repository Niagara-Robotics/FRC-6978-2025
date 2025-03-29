// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStation.h>

#include <iostream>

void Robot::StartCompetition() {
    std::cout << "Started Competition" << std::endl;
    drive_scheduler->register_task(swerve_controller);
    input_scheduler->register_task(&driver_input);
    input_scheduler->register_task(&operator_input);

    drive_scheduler->register_task(&auto_pilot);

    tracking_scheduler->register_task(&tracking);
    tracking_scheduler->register_task(&intake);
    tracking_scheduler->register_task(&lift);

    //grpl::start_can_bridge();

    HAL_ObserveUserProgramStarting();
    HAL_ObserveUserProgramDisabled();
    std::cout << "Tasks registered\n";

    frc::DataLogManager::Start();
    frc::DataLogManager::LogNetworkTables(true);

    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

    std::cout << "Started logging\n";

    while (true)
    {
        HAL_RefreshDSData();
        frc::DriverStation::RefreshData();
        HAL_ControlWord word;
        HAL_GetControlWord(&word);
        bool enabled = word.enabled;
        if(word.test) enabled = false;

        drive_scheduler->set_robot_status(enabled, word.autonomous);
        tracking_scheduler->set_robot_status(enabled, word.autonomous);
        input_scheduler->set_robot_status(enabled, word.autonomous);

        if(enabled && !word.autonomous) {HAL_ObserveUserProgramTeleop();}
        else if(enabled && word.autonomous) {HAL_ObserveUserProgramAutonomous();}
        else {HAL_ObserveUserProgramDisabled();}

        global_fault_manager->refresh();

        frc::SmartDashboard::UpdateValues();
        nt::NetworkTableInstance(nt::GetDefaultInstance()).Flush();
        
        usleep(12000);
        if(should_exit) {
            break;
        }
    }

    frc::DataLogManager::Stop();

    drive_scheduler->~Scheduler();
    tracking_scheduler->~Scheduler();
    input_scheduler->~Scheduler();
}

void Robot::EndCompetition() {
    std::cout << "ended\n";
    should_exit = true;
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
