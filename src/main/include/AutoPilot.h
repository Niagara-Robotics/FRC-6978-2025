#pragma once
#include "Task.h"
#include <frc/geometry/Pose2d.h>
#include "ControlChannel.h"
#include "SwerveController.h"
#include "Tracking.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>


#include <pathplanner/lib/auto/AutoBuilder.h>

#include <iostream>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;

enum class AutoPilotTwistMode {
    none,
    heading,
    face,
    pose,
    speaker,
    planner
};

enum class AutoPilotTranslateMode {
    none,
    point,
    planner
};

class AutoPilot: public Task, frc2::Subsystem
{
private:
    units::angle::radian_t heading_target = 90_deg;

    controlchannel::ControlHandle<LateralSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;
    Tracking *tracking;
    SwerveController *swerve_controller;

    units::angular_velocity::radians_per_second_t heading_proportional(units::angle::radian_t target, units::angle::radian_t current);
    frc2::Subsystem subsystem = frc2::Subsystem();

    frc::SendableChooser<frc2::CommandPtr*> auto_chooser;
    frc2::Command *current_auto_command;

    bool auto_initialized = false;
    bool auto_running = false;

public:
    controlchannel::ControlChannel<AutoPilotTwistMode> twist_mode_channel = controlchannel::ControlChannel<AutoPilotTwistMode>(AutoPilotTwistMode::none);
    controlchannel::ControlChannel<units::angle::radian_t> heading_channel = controlchannel::ControlChannel<units::angle::radian_t>(0_rad);
    
    
    controlchannel::ControlChannel<frc::Pose2d> pose_channel = controlchannel::ControlChannel<frc::Pose2d>(frc::Pose2d());

    AutoPilot(
        controlchannel::ControlHandle<LateralSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        Tracking *tracking,
        SwerveController *swerve_controller);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void drive_robot_relative(frc::ChassisSpeeds speeds);

    

    void set_rotation_mode(AutoPilotTwistMode mode);
    void set_hdg_target(units::angle::radian_t target);

    ~AutoPilot(){};
};