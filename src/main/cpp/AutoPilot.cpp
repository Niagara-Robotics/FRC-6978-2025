#include "AutoPilot.h"
#include <frc/smartdashboard/SmartDashboard.h>

//#include <pathplanner/lib/auto/AutoBuilder.h>

#include <iostream>

//#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
//#include <pathplanner/lib/auto/NamedCommands.h>
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
        SwerveController *swerve_controller):
        planar_handle(planar_handle), twist_handle(twist_handle), tracking(tracking), swerve_controller(swerve_controller), lift_handle(lift_handle)
{   

    /*AutoBuilder::configure(
        [this, tracking](){ return tracking->get_pose(); }, // Robot pose supplier
        [this, tracking](frc::Pose2d pose){ tracking->reset_pose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this, tracking](){ return tracking->get_chassis_speeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this, twist_handle, planar_handle](auto speeds){ drive_robot_relative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        pathplanner::PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(2.0, 0.0, 0.0), 0.004_s // Rotation PID constants
        ),
        RobotConfig(52_kg, 10_kg_sq_m,
			ModuleConfig(4_in, 5_mps, 1.0, frc::DCMotor(12_V, 4.6_nm, 260_A, 1.5_A, 100_tps, 1), 40_A,1), 
            swerve_controller->fetch_module_offsets()),
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
    );*/
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
    
    case AutoPilotTwistMode::none:
        twist_handle.set(0_rad_per_s);
        break;
    case AutoPilotTwistMode::planner:

        break;
    }

    switch (lateral_mode_channel.get())
    {
    case AutoPilotTranslateMode::none:
        planar_handle.release();
        break;
    case AutoPilotTranslateMode::point:
        planar_handle.try_take_control();
        planar_handle.set(point_proportional(frc::Pose2d(frc::Translation2d(4.979_m,5.19_m), frc::Rotation2d()), tracking->get_pose()));
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