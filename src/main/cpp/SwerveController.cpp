#include "SwerveController.h"
#include <iostream>

#include <frc/DigitalOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <thread>
#include <future>
#include <sched.h>
#include "ControlChannel.h"

SwerveController::SwerveController(GlobalFaultManager *global_fm)
{
    module_positions_publisher = nt::StructArrayTopic<frc::SwerveModulePosition>(nt::GetTopic(nt::GetDefaultInstance(), "/swerve/module_positions")).Publish();
    for (size_t i = 0; i < 4; i++) {
        global_fm->register_manager(&modules[i]->fault_manager);
    }
    return;
}

void SwerveController::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(2000);
}

frc::SwerveDriveKinematics<4> SwerveController::get_kinematics() {
    return kinematics;
}

void SwerveController::call(bool robot_enabled, bool autonomous) {
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

    LateralSwerveRequest lateral_request = planar_velocity_channel.get();

    target_chassis_speeds.vx = lateral_request.x;
    target_chassis_speeds.vy = lateral_request.y;
    target_chassis_speeds.omega = twist_velocity_channel.get();

    if(planar_velocity_channel.has_control(-1)) {
        target_chassis_speeds.vx = 0_mps;
        target_chassis_speeds.vy = 0_mps;
    }

    if(twist_velocity_channel.has_control(-1)) {
        target_chassis_speeds.omega = 0_rad_per_s;
    }

    switch (lateral_request.request_type) {
        case full:
            target_states = kinematics.ToSwerveModuleStates(
                frc::ChassisSpeeds::FromFieldRelativeSpeeds(target_chassis_speeds, current_rotation), 
                frc::Translation2d(0.0_m,0.0_m));
            break;
        case full_robot_relative:
            target_states = kinematics.ToSwerveModuleStates(target_chassis_speeds, frc::Translation2d(0.0_m,0.0_m));
            break;
        default:
            target_states[0] = frc::SwerveModuleState();
            target_states[1] = frc::SwerveModuleState();
            target_states[2] = frc::SwerveModuleState();
            target_states[3] = frc::SwerveModuleState();
            break;
    }
    

    for (size_t i = 0; i < 4; i++)
    {
        if(robot_enabled) {
            modules[i]->apply(target_states[i]);
            //modules[i]->test_couple();
            
        }
        else  {
            modules[i]->idle();
        }
        if(!skip_safety) modules[i]->fault_check();
    }

    skip_safety = !skip_safety;

    module_positions_publisher.Set(last_reported_positions);

    std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now() - start_time;

    frc::SmartDashboard::PutNumber("swerve_controller_total_time_percent", (diff.count() / 2000.0) * 100.0);

    for (int i = 0; i < 4; i++)
    {
        frc::SmartDashboard::PutNumber("smc" + std::to_string(i) + "_status", modules[i]->get_state());
    }

    //fake_talon.SetControl(ctre::phoenix6::controls::VoltageOut(1.1_V));
}

void SwerveController::set_chassis_rotation(frc::Rotation2d rotation) {
    current_rotation = rotation;
    frc::SmartDashboard::PutNumber("swerveAngle", rotation.Radians().value());
}

wpi::array<frc::SwerveModulePosition, 4> SwerveController::fetch_module_positions() {
    for (size_t i = 0; i < 4; i++) {
        last_reported_positions[i] = modules[i]->get_position();
    }
    return last_reported_positions;
}

frc::ChassisSpeeds SwerveController::get_chassis_speeds() {
    wpi::array<frc::SwerveModuleState, 4> states= {
        frc::SwerveModuleState{0_mps, frc::Rotation2d()},
        frc::SwerveModuleState{0_mps, frc::Rotation2d()},
        frc::SwerveModuleState{0_mps, frc::Rotation2d()},
        frc::SwerveModuleState{0_mps, frc::Rotation2d()}
    };
    for (size_t i = 0; i < 4; i++) {
        states[i] = modules[i]->get_module_state();
    }
    return kinematics.ToChassisSpeeds(states);
}

std::vector<frc::Translation2d> SwerveController::fetch_module_offsets() {
    std::vector<frc::Translation2d> positions = std::vector<frc::Translation2d>();
    for (size_t i = 0; i < 4; i++) {
        positions.push_back(module_positions[i]);
    }
    return positions;
}

bool SwerveController::is_paused() {
    return false;
}

SwerveController::~SwerveController()
{
}

