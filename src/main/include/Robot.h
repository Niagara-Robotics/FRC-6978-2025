// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Scheduler.h"
#include "SwerveController.h"
#include "InputTest.h"

class Robot: frc::RobotBase {
    public:
    void StartCompetition() override;
    void EndCompetition() override;
    Scheduler *teleScheduler = new Scheduler("teleop");
    InputTest *input_test = new InputTest();
    SwerveController *swerve_controller = new SwerveController(input_test);
    bool should_exit = false;
};
