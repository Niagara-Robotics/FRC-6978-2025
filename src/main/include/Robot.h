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
#include "AutoPilot.h"

#include "FaultManager.h"

#include "OperatorInput.h"

#include <grpl/CanBridge.h>
#include <grpl/LaserCan.h>

#include "Intake.h"
#include "Lift.h"


class Robot: frc::RobotBase {
    public:
    void StartCompetition() override;
    void EndCompetition() override;
    
    GlobalFaultManager *global_fault_manager = new GlobalFaultManager();

    Scheduler *drive_scheduler = new Scheduler("teleop");
    Scheduler *tracking_scheduler = new Scheduler("tracking");
    Scheduler *input_scheduler = new Scheduler("input");

    SwerveController *swerve_controller = new SwerveController(global_fault_manager);

    Tracking tracking = Tracking(swerve_controller);

    intake::Intake intake = intake::Intake(global_fault_manager);

    Lift lift = Lift(&intake, swerve_controller->planar_velocity_channel.get_handle(), global_fault_manager);

    AutoPilot auto_pilot = AutoPilot(
        swerve_controller->planar_velocity_channel.get_handle(), 
        swerve_controller->twist_velocity_channel.get_handle(),
        lift.target_mechanism_state.get_handle(),
        &tracking,
        swerve_controller,
        global_fault_manager
    );

    DriverInput driver_input = DriverInput(
        swerve_controller->planar_velocity_channel.get_handle(), 
        swerve_controller->twist_velocity_channel.get_handle(),
        intake.intake_action_channel.get_handle(),
        lift.target_mechanism_state.get_handle(),
        auto_pilot.lateral_mode_channel.get_handle(),
        auto_pilot.twist_mode_channel.get_handle(),
        global_fault_manager,
        &tracking
    );

    OperatorInput operator_input = OperatorInput(
        swerve_controller->planar_velocity_channel.get_handle(), 
        swerve_controller->twist_velocity_channel.get_handle(),
        intake.intake_action_channel.get_handle(),
        lift.target_mechanism_state.get_handle(),
        lift.target_place_position.get_handle(),
        global_fault_manager,
        &tracking
    );

    bool should_exit = false;
};
