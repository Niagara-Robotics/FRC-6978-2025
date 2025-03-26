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

//using namespace pathplanner;

AutoPilot::AutoPilot(controlchannel::ControlHandle<LateralSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<LiftMechanismState> lift_handle,
        Tracking *tracking,
        SwerveController *swerve_controller,
        GlobalFaultManager *global_fm):
        planar_handle(planar_handle), twist_handle(twist_handle), tracking(tracking), swerve_controller(swerve_controller), lift_handle(lift_handle)
{   
    global_fm->register_manager(&fault_manager);
    AutoBuilder::configure(
        (std::function<frc::Pose2d()>) [this, tracking](){ return tracking->get_pose(); }, // Robot pose supplier
        (std::function<void(const frc::Pose2d&)>) [this, tracking](frc::Pose2d pose){ tracking->reset_pose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        (std::function<frc::ChassisSpeeds()>) [this, tracking](){ return tracking->get_chassis_speeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (std::function<void(const frc::ChassisSpeeds&)>) [this, twist_handle, planar_handle](auto speeds){ drive_robot_relative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::shared_ptr<pathplanner::PPHolonomicDriveController>(new pathplanner::PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(2.0, 0.0, 0.0), 0.004_s // Rotation PID constants
        )),
        RobotConfig(52_kg, 10_kg_sq_m,
			ModuleConfig(4_in, 5_mps, 1.0, frc::DCMotor(12_V, 4.6_Nm, 260_A, 1.5_A, 628_rad_per_s, 1), 40_A,1), 
            swerve_controller->fetch_module_offsets()),
        (std::function<bool()>) []() {
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
}

void AutoPilot::drive_robot_relative(frc::ChassisSpeeds speeds) {
    twist_handle.try_take_control();
    planar_handle.try_take_control();
    //lock out external control from the twist mode channel
    twist_mode_channel.take_control(0, false);
    twist_mode_channel.set(0, AutoPilotTwistMode::planner);

    twist_handle.set(speeds.omega);
    planar_handle.set(LateralSwerveRequest(speeds.vx, speeds.vy, SwerveRequestType::full));
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

LateralSwerveRequest AutoPilot::point_proportional(frc::Pose2d target, frc::Pose2d current) {
    units::meter_t deltaX = (target.X() - current.X());
    units::meter_t deltaY = (target.Y() - current.Y());

    units::meters_per_second_t xOut = deltaX * (7.5_mps / 1_m);

    units::meters_per_second_t yOut = deltaY * (7.5_mps / 1_m);

    units::meters_per_second_t max = 1.0_mps;

    xOut = (xOut > max)? max: xOut;
    xOut = (xOut < -max)? -max: xOut;

    yOut = (yOut > max)? max: yOut;
    yOut = (yOut < -max)? -max: yOut;

    if(sqrt(pow(deltaX.value(), 2) + pow(deltaY.value(), 2)) < 0.025) {
        xOut = 0_mps;
        yOut = 0_mps;
    }

    return LateralSwerveRequest(xOut, yOut, SwerveRequestType::full);
}

void AutoPilot::call(bool robot_enabled, bool autonomous) {
    frc::SmartDashboard::PutBoolean("auto_running", auto_running);
    frc::SmartDashboard::PutBoolean("auto_initialized", auto_initialized);
    auto alliance = frc::DriverStation::GetAlliance();

    if(!alliance.has_value()) {
        fault_manager.add_fault(Fault(false, FaultIdentifier::allianceUnavailable));
    } else {
        fault_manager.clear_fault(Fault(false, FaultIdentifier::allianceUnavailable));
    }

    frc::Pose2d face_pose;

    reef_face_channel.take_control(0, false);
    if((int)reef_face_channel.get() < 0) reef_face_channel.set(0, (ReefFace)0);
    if((int)reef_face_channel.get() > 5) reef_face_channel.set(0, (ReefFace)5);
    if(alliance.has_value())
        face_pose = (alliance.value() == frc::DriverStation::Alliance::kBlue)? blue_reef_centers[(int)reef_face_channel.get()]: red_reef_centers[(int)reef_face_channel.get()];
    else
        face_pose = blue_reef_centers[(int)reef_face_channel.get()];

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
    case AutoPilotTwistMode::reef: {
        twist_handle.try_take_control();
        twist_handle.set(heading_proportional(face_pose.Rotation().Radians() - 180_deg, tracking->get_pose().Rotation().Radians()));
        break;
    }
    
    case AutoPilotTwistMode::none:
        twist_handle.set(0_rad_per_s);
        break;
    case AutoPilotTwistMode::planner:

        break;
    }

    
    frc::Translation2d final_translation;

    switch (lateral_mode_channel.get())
    {
    case AutoPilotTranslateMode::none:
        planar_handle.release();
        break;
    case AutoPilotTranslateMode::reef:
        //calculate the position
        final_translation = face_pose.Translation() + frc::Translation2d(whisker_clear_distance, 0_m).RotateBy(face_pose.Rotation());
        final_translation = final_translation + frc::Translation2d(0_m, ((int)reef_tree_channel.get()>0)? whisker_strafe_distance: -whisker_strafe_distance).RotateBy(face_pose.Rotation());

        frc::SmartDashboard::PutNumber("whiskerPoseX", final_translation.X().value());
        frc::SmartDashboard::PutNumber("whiskerPoseY", final_translation.Y().value());

        reef_face_channel.take_control(0, true);
        planar_handle.try_take_control();
        planar_handle.set(point_proportional(frc::Pose2d(final_translation, frc::Rotation2d()), tracking->get_pose()));
        break;
    default:
        break;
    }

    if(!robot_enabled) {
        auto_running = false;
        planar_handle.release();
        return;
    }
    if(autonomous) {
        if(!auto_running && !auto_initialized) {
            auto_running = true;
            auto_initialized = true;
            planar_handle.try_take_control();
            planar_handle.set(LateralSwerveRequest(0.5_mps, 0.0_mps, SwerveRequestType::full_robot_relative));
            auto_start = std::chrono::steady_clock::now();
            lift_handle.try_take_control();
            lift_handle.set(LiftMechanismState::mid);
        }

        if(std::chrono::steady_clock::now() - auto_start > std::chrono::milliseconds(2500) && auto_running) {
            planar_handle.release();
            auto_running = false;
        }

        return;
    } else if(auto_running) {
        auto_running = false;
        planar_handle.release();
    }
}

void AutoPilot::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(4000);
}

bool AutoPilot::is_paused() {
    return false;
}