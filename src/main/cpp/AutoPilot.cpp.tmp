#include "AutoPilot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>

#include <iostream>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

using namespace pathplanner;

AutoPilot::AutoPilot(controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<IntakeIndexingMode> index_mode_handle,
        controlchannel::ControlHandle<LauncherMode> launcher_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle,
        controlchannel::ControlHandle<AutoShotMode> auto_shot_mode_handle,
        Tracking *tracking, NoteHandler *note_handler):
        planar_handle(planar_handle), twist_handle(twist_handle), tracking(tracking), index_mode_handle(index_mode_handle),
        launcher_mode_handle(launcher_mode_handle), launcher_tilt_handle(launcher_tilt_handle), note_handler(note_handler), auto_shot_mode_handle(auto_shot_mode_handle)
{
    AutoBuilder::configureHolonomic(
        [this, tracking](){ return tracking->get_pose(); }, // Robot pose supplier
        [this, tracking](frc::Pose2d pose){ tracking->reset_pose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this, tracking](){ return tracking->get_chassis_speeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this, twist_handle, planar_handle](auto speeds){ drive_robot_relative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        HolonomicPathFollowerConfig( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(2.0, 0.0, 0.0), 5.0_mps, 0.438_m, ReplanningConfig(), 0.004_s // Rotation PID constants
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kBlue;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    NamedCommands::registerCommand("close_shot", std::move(frc2::FunctionalCommand(
        [this]() { close_shot_init(); },
        [this]() { close_shot_exec(); },
        [this](bool interrupted) { close_shot_kill(interrupted); },
        [this]() { return close_shot_complete(); }
    ).ToPtr()));

    NamedCommands::registerCommand("pickup", std::move(frc2::FunctionalCommand(
        [this]() { pickup_init(); },
        [this]() { pickup_exec(); },
        [this](bool interrupted) { pickup_kill(interrupted); },
        [this]() { return pickup_complete(); }
    ).ToPtr()));

    NamedCommands::registerCommand("auto_shot", std::move(frc2::FunctionalCommand(
        [this]() { auto_shot_init(); },
        [this]() { auto_shot_exec(); },
        [this](bool interrupted) { auto_shot_kill(interrupted); },
        [this]() { return auto_shot_complete(); }
    ).ToPtr()));

    centre_auto = AutoBuilder::buildAuto("centre");
    exit_auto = AutoBuilder::buildAuto("Far Exit");
    exit_auto = AutoBuilder::buildAuto("right exit");

    auto_chooser.SetDefaultOption("centre", &centre_auto);
    auto_chooser.SetDefaultOption("left exit", &exit_auto);
    auto_chooser.SetDefaultOption("right exit", &right_exit_auto);
    auto_chooser.SetDefaultOption("none", nullptr);
    frc::SmartDashboard::PutData("auto", &auto_chooser);
    
}

void AutoPilot::drive_robot_relative(frc::ChassisSpeeds speeds) {
    twist_handle.try_take_control();
    planar_handle.try_take_control();
    //lock out external control from the twist mode channel
    twist_mode_channel.take_control(0, false);
    twist_mode_channel.set(0, AutoPilotTwistMode::planner);

    twist_handle.set(speeds.omega);
    planar_handle.set(PlanarSwerveRequest(speeds.vx, speeds.vy, true));
}

units::angular_velocity::radians_per_second_t AutoPilot::heading_proportional(units::angle::radian_t target, units::angle::radian_t current) {
    units::angle::radian_t diff = (current - target);
    if(diff.value() > M_PI) diff-=M_PI*2_rad;
    if(diff.value() < -M_PI) diff+=M_PI*2_rad;

    units::angular_velocity::radians_per_second_t output = (-diff)* 5.8_rad_per_s/1_rad;
    if(output> 3.0_rad_per_s) output = 3.0_rad_per_s;
    if(output< -3.0_rad_per_s) output = -3.0_rad_per_s;
    if(fabs(output.value()) < 0.005) output = 0_rad_per_s;
    return output;
}

void AutoPilot::call(bool robot_enabled, bool autonomous) {
    frc::SmartDashboard::PutBoolean("auto_running", auto_running);
    frc::SmartDashboard::PutBoolean("auto_initialized", auto_initialized);

    switch (twist_mode_channel.get())
    {
    case AutoPilotTwistMode::heading: {//heading PID
        twist_handle.try_take_control();
        twist_handle.set(heading_proportional(heading_channel.get(), tracking->get_pose().Rotation().Radians()));
        break;
    }

    case AutoPilotTwistMode::face: {
        twist_handle.try_take_control();
        frc::Translation2d diff_translation = pose_channel.get().Translation() - tracking->get_pose().Translation();
        double target = atan2(diff_translation.Y().value(), diff_translation.X().value());
        frc::SmartDashboard::PutNumber("autopilot/target_heading", target);
        twist_handle.set(heading_proportional(target * 1.0_rad, tracking->get_pose().Rotation().Radians()));
        break;
    }

    case AutoPilotTwistMode::speaker: {
        if(std::chrono::steady_clock::now() < (tracking->get_speaker_pose().observation_time + std::chrono::milliseconds(250))) {
            twist_handle.try_take_control();
            twist_handle.set(heading_proportional(0.0_rad, tracking->get_speaker_pose().heading));
        } else {
            twist_handle.set(0_rad_per_s);
        }
        break;
    }
    
    case AutoPilotTwistMode::none:
        twist_handle.set(0_rad_per_s);
        break;
    case AutoPilotTwistMode::planner:

        break;
    }

    if(!robot_enabled) {
        auto_running = false;
        return;
    }
    if(autonomous) {
        frc::SmartDashboard::PutBoolean("null_auto", false);
        if(!auto_initialized) {
            if(auto_chooser.GetSelected() == nullptr) {
                frc::SmartDashboard::PutBoolean("null_auto", true);
                return;
            }
            current_auto_command = auto_chooser.GetSelected()->get();
            current_auto_command->Schedule();
            current_auto_command->Initialize();
            current_auto_command->Execute();
            auto_initialized = true;
            auto_running = true;
            return;
        }
        if(current_auto_command->IsFinished()) return;
        current_auto_command->Execute();
        return;
    }
    if(auto_running) {
        current_auto_command->End(true);
        twist_mode_channel.set(0, AutoPilotTwistMode::none);
        twist_mode_channel.take_control(0, true); //allow others to grab the handle
        planar_handle.set(PlanarSwerveRequest(0_mps,0_mps));
        auto_running = false;
    }
}

void AutoPilot::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(4000);
}

bool AutoPilot::is_paused() {
    return false;
}