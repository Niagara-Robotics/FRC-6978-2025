// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Scheduler.h"
#include "SwerveController.h"
#include "GyroInput.h"
#include "DriverInput.h"
#include "Tracking.h"

#include "FaultManager.h"

#include "OperatorInput.h"

#include <grpl/CanBridge.h>


class Robot: frc::RobotBase {
    public:
    void StartCompetition() override;
    void EndCompetition() override;
    
    GlobalFaultManager *global_fault_manager = new GlobalFaultManager();

    Scheduler *drive_scheduler = new Scheduler("teleop");
    Scheduler *tracking_scheduler = new Scheduler("tracking");
    Scheduler *input_scheduler = new Scheduler("input");

    SwerveController *swerve_controller = new SwerveController();

    Tracking tracking = Tracking(swerve_controller);

    DriverInput driver_input = DriverInput(
        swerve_controller->planar_velocity_channel.get_handle(), 
        swerve_controller->twist_velocity_channel.get_handle(),
        global_fault_manager,
        &tracking
    );

    OperatorInput operator_input = OperatorInput(
        swerve_controller->planar_velocity_channel.get_handle(), 
        swerve_controller->twist_velocity_channel.get_handle(),
        &tracking
    );

    bool should_exit = false;
};
