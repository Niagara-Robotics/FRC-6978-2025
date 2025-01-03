#pragma once
#include "Task.h"
#include <frc/geometry/Pose2d.h>
#include "ControlChannel.h"
#include "SwerveController.h"
#include "Tracking.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include "NoteHandler.h"
#include "AutoShot.h"

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

    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;
    Tracking *tracking;
    NoteHandler *note_handler;

    controlchannel::ControlHandle<IntakeIndexingMode> index_mode_handle;
    controlchannel::ControlHandle<LauncherMode> launcher_mode_handle;

    controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle;
    controlchannel::ControlHandle<AutoShotMode> auto_shot_mode_handle;

    units::angular_velocity::radians_per_second_t heading_proportional(units::angle::radian_t target, units::angle::radian_t current);
    frc2::Subsystem subsystem = frc2::Subsystem();

    frc::SendableChooser<frc2::CommandPtr*> auto_chooser;

    frc2::CommandPtr centre_auto = std::move(frc2::FunctionalCommand(
        [this]() { //init
            close_shot_init();
        },
        [this]() { //execute
            close_shot_exec();
        },
        [this](bool interrupted) { //stop
            close_shot_kill(interrupted);
        },
        [this]() { //is finished
            return close_shot_complete();
        }
    ).ToPtr());

    frc2::CommandPtr exit_auto = std::move(frc2::FunctionalCommand(
        [this]() { //init
            close_shot_init();
        },
        [this]() { //execute
            close_shot_exec();
        },
        [this](bool interrupted) { //stop
            close_shot_kill(interrupted);
        },
        [this]() { //is finished
            return close_shot_complete();
        }
    ).ToPtr());
    frc2::CommandPtr right_exit_auto = std::move(frc2::FunctionalCommand(
        [this]() { //init
            close_shot_init();
        },
        [this]() { //execute
            close_shot_exec();
        },
        [this](bool interrupted) { //stop
            close_shot_kill(interrupted);
        },
        [this]() { //is finished
            return close_shot_complete();
        }
    ).ToPtr());

    frc2::Command *current_auto_command;

    bool auto_initialized = false;
    bool auto_running = false;

    void close_shot_init();
    void close_shot_exec();
    bool close_shot_complete();
    void close_shot_kill(bool interrupt);

    void pickup_init();
    void pickup_exec();
    bool pickup_complete();
    void pickup_kill(bool interrupt);

    void auto_shot_init();
    void auto_shot_exec();
    bool auto_shot_complete();
    void auto_shot_kill(bool interrupt);

public:
    controlchannel::ControlChannel<AutoPilotTwistMode> twist_mode_channel = controlchannel::ControlChannel<AutoPilotTwistMode>(AutoPilotTwistMode::none);
    controlchannel::ControlChannel<units::angle::radian_t> heading_channel = controlchannel::ControlChannel<units::angle::radian_t>(0_rad);
    
    
    controlchannel::ControlChannel<frc::Pose2d> pose_channel = controlchannel::ControlChannel<frc::Pose2d>(frc::Pose2d());

    AutoPilot(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<IntakeIndexingMode> index_mode_handle,
        controlchannel::ControlHandle<LauncherMode> launcher_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle,
        controlchannel::ControlHandle<AutoShotMode> auto_shot_mode_handle,
        Tracking *tracking,
        NoteHandler *note_handler);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void drive_robot_relative(frc::ChassisSpeeds speeds);

    

    void set_rotation_mode(AutoPilotTwistMode mode);
    void set_hdg_target(units::angle::radian_t target);

    ~AutoPilot(){};
};