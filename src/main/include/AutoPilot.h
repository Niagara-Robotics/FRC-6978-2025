#pragma once
#include "Task.h"
#include "ControlChannel.h"
#include "SwerveController.h"
#include "Tracking.h"
#include "Lift.h"
#include "FaultManager.h"

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>

#include <pathplanner/lib/auto/AutoBuilder.h>

#include <iostream>
#include <chrono>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;

enum class AutoPilotTwistMode {
    none,
    heading,
    face,
    pose,
    reef,
    planner
};

enum class AutoPilotTranslateMode {
    none,
    reef,
    reef_long,
    planner
};

enum class ReefFace {
    AB = 0,
    CD = 1,
    EF = 2,
    GH = 3,
    IJ = 4,
    KL = 5
};

enum class ReefTree {
    left,
    right
};

class LiftMidCommand: public frc2::CommandHelper<frc2::Command, LiftMidCommand> {
    Lift *lift;

    controlchannel::ControlHandle<LiftMechanismState> action_handle;

public:
    LiftMidCommand(Lift *lift): 
        lift(lift), 
        action_handle(lift->target_mechanism_state.get_handle()){};

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
};

class L4PlaceCommand: public frc2::CommandHelper<frc2::Command, L4PlaceCommand> {
    Lift *lift;

    controlchannel::ControlHandle<LiftMechanismState> action_handle;
    controlchannel::ControlHandle<int> level_handle;

public:
    L4PlaceCommand(Lift *lift): 
        lift(lift), 
        action_handle(lift->target_mechanism_state.get_handle()),
        level_handle(lift->target_place_position.get_handle()){};


    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
};

class UnmaskVisionCommand: public frc2::CommandHelper<frc2::Command, UnmaskVisionCommand> {
    Lift *lift;
    controlchannel::ControlHandle<bool> mask_handle;

public:
    UnmaskVisionCommand(Tracking *tracking): 
        mask_handle(tracking->mask_vision_channel.get_handle()){};

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
};



class AutoPilot: public Task, frc2::Subsystem
{
private:
    units::angle::radian_t heading_target = 90_deg;

    controlchannel::ControlHandle<LateralSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;

    controlchannel::ControlHandle<LiftMechanismState> lift_handle;
    controlchannel::ControlHandle<bool> vision_mask_handle;

    Tracking *tracking;
    SwerveController *swerve_controller;
    Lift *lift;

    nt::StructPublisher<frc::Pose2d> target_pose_publisher;

    LateralSwerveRequest point_proportional(frc::Pose2d target, frc::Pose2d current);

    units::angular_velocity::radians_per_second_t heading_proportional(units::angle::radian_t target, units::angle::radian_t current);
    frc2::Subsystem subsystem = frc2::Subsystem();

    frc::SendableChooser<frc2::Command*> auto_chooser;
    frc2::Command *current_auto_command;

    std::chrono::time_point<std::chrono::steady_clock> auto_start;

    frc::Pose2d red_reef_centers[6] = {
        frc::Pose2d(546.87_in, 158.50_in, frc::Rotation2d(0_rad)), //AB
        frc::Pose2d(530.49_in, 186.83_in, frc::Rotation2d(60_deg)), //CD
        frc::Pose2d(497.77_in, 186.83_in, frc::Rotation2d(120_deg)), //EF
        frc::Pose2d(481.39_in, 158.50_in, frc::Rotation2d(180_deg)), //GH
        frc::Pose2d(497.77_in, 130.17_in, frc::Rotation2d(240_deg)), //IJ
        frc::Pose2d(530.49_in, 130.17_in, frc::Rotation2d(300_deg)) //KL
    };

    frc::Pose2d blue_reef_centers[6] = {
        frc::Pose2d(144.0_in, 158.5_in, frc::Rotation2d(180_deg)), //AB
        frc::Pose2d(160.39_in, 130.17_in, frc::Rotation2d(240_deg)), //CD
        frc::Pose2d(193.1_in, 130.17_in, frc::Rotation2d(300_deg)), //EF
        frc::Pose2d(209.49_in, 158.5_in, frc::Rotation2d(0_deg)), //GH
        frc::Pose2d(193.1_in, 186.83_in, frc::Rotation2d(60_deg)), //IJ
        frc::Pose2d(160.39_in, 186.83_in, frc::Rotation2d(120_deg)), //AB
    };

    units::length::meter_t whisker_clear_distance = 15_in;
    units::length::meter_t whisker_long_distance = 18.5_in;
    units::length::meter_t whisker_strafe_distance = 7.5_in;
    units::length::meter_t side_offset = 1.5_in;

    double delta = 0;

    bool auto_initialized = false;
    bool auto_running = false;

    AutoPilotTranslateMode last_lateral_mode = AutoPilotTranslateMode::none;
    AutoPilotTwistMode last_twist_mode = AutoPilotTwistMode::none;

    FaultManager fault_manager = FaultManager("AutoPilot");

public:
    controlchannel::ControlChannel<AutoPilotTwistMode> twist_mode_channel = controlchannel::ControlChannel<AutoPilotTwistMode>(AutoPilotTwistMode::none);
    controlchannel::ControlChannel<AutoPilotTranslateMode> lateral_mode_channel = controlchannel::ControlChannel<AutoPilotTranslateMode>(AutoPilotTranslateMode::none);

    controlchannel::ControlChannel<units::angle::radian_t> heading_channel = controlchannel::ControlChannel<units::angle::radian_t>(-2.113_rad);
    
    
    controlchannel::ControlChannel<ReefFace> reef_face_channel = controlchannel::ControlChannel<ReefFace>(ReefFace::KL);
    controlchannel::ControlChannel<ReefTree> reef_tree_channel = controlchannel::ControlChannel<ReefTree>(ReefTree::left);

    controlchannel::ControlChannel<frc::Pose2d> pose_channel = controlchannel::ControlChannel<frc::Pose2d>(frc::Pose2d());

    AutoPilot(
        controlchannel::ControlHandle<LateralSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<LiftMechanismState> lift_handle,
        Tracking *tracking,
        SwerveController *swerve_controller,
        Lift *lift,
        GlobalFaultManager *global_fm);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void drive_robot_relative(frc::ChassisSpeeds speeds);

    

    void set_rotation_mode(AutoPilotTwistMode mode);
    void set_hdg_target(units::angle::radian_t target);

    bool is_finished();

    ~AutoPilot(){};
};

class AutoAlignLeftCommand: public frc2::CommandHelper<frc2::Command, AutoAlignLeftCommand> {
    AutoPilot *auto_pilot;

    controlchannel::ControlHandle<AutoPilotTranslateMode> lateral_handle;
    controlchannel::ControlHandle<AutoPilotTwistMode> twist_handle;
    controlchannel::ControlHandle<ReefTree> tree_handle;

public:
    AutoAlignLeftCommand(AutoPilot *auto_pilot): 
        auto_pilot(auto_pilot), 
        lateral_handle(auto_pilot->lateral_mode_channel.get_handle()),
        twist_handle(auto_pilot->twist_mode_channel.get_handle()),
        tree_handle(auto_pilot->reef_tree_channel.get_handle()){};


    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
};