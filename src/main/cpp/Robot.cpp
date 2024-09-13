// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStation.h>

#include <iostream>

void Robot::StartCompetition() {
  std::cout << "started";
  teleScheduler->register_task(input_test);
  teleScheduler->register_task(swerve_controller);
  HAL_ObserveUserProgramStarting();
  HAL_ObserveUserProgramDisabled();
  std::cout << "started\n";
    while (true)
    {
        HAL_RefreshDSData();
        HAL_ControlWord word;
        HAL_GetControlWord(&word);
        swerve_controller->notify_enabled(word.enabled);
        
        
        usleep(5000);
        if(should_exit) {
          break;
        }
    }
    teleScheduler->~Scheduler();
}

void Robot::EndCompetition() {
  std::cout << "ended";
  should_exit = true;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
