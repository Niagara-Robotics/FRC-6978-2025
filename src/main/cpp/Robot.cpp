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

    tracking_scheduler->register_task(&tracking);

    HAL_ObserveUserProgramStarting();
    HAL_ObserveUserProgramDisabled();
    std::cout << "Tasks registered\n";
    while (true)
    {
        HAL_RefreshDSData();
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

        frc::SmartDashboard::UpdateValues();
        usleep(9000);
        if(should_exit) {
            break;
        }
    }
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
