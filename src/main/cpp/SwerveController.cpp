#include "SwerveController.h"
#include <iostream>

#include <frc/DigitalOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <thread>
#include <future>
#include <sched.h>
#include "ControlChannel.h"

SwerveController::SwerveController()
{
    module_positions_publisher = nt::StructArrayTopic<frc::SwerveModulePosition>(nt::GetTopic(nt::GetDefaultInstance(), "swerve/module_positions")).Publish();
    return;
}

void SwerveController::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(1880);
}

frc::SwerveDriveKinematics<4> SwerveController::get_kinematics() {
    return kinematics;
}

void SwerveController::call(bool robot_enabled, bool autonomous) {
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

    PlanarSwerveRequest planar_request = planar_velocity_channel.get();

    target_chassis_speeds.vx = planar_request.x;
    target_chassis_speeds.vy = planar_request.y;
    target_chassis_speeds.omega = twist_velocity_channel.get();
    
    wpi::array<frc::SwerveModuleState, 4> states = (planar_request.robot_relative)? kinematics.ToSwerveModuleStates(target_chassis_speeds, frc::Translation2d(0.0_m,0.0_m)) : kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(target_chassis_speeds, current_rotation), frc::Translation2d(0.0_m,0.0_m));

    double max_apply_time = 0;
    std::list<std::future<void>> futures = std::list<std::future<void>>();

    frc::SwerveModuleState test_state = frc::SwerveModuleState();
    

    for (size_t i = 0; i < 4; i++)
    {
        if(robot_enabled) {
            modules[i]->apply(states[i]);
        }
        else  {
            modules[i]->idle();
        }
    }

    module_positions_publisher.Set(last_reported_positions);

    std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now() - start_time;

    frc::SmartDashboard::PutNumber("swerve_controller_total_time_percent", (diff.count() / 1880.0) * 100.0);

    for (int i = 0; i < 4; i++)
    {
        frc::SmartDashboard::PutNumber("smc" + std::to_string(i) + "_status", modules[i]->get_state());
    }

    //fake_talon.SetControl(ctre::phoenix6::controls::VoltageOut(1.1_V));
}

void SwerveController::set_chassis_rotation(frc::Rotation2d rotation) {
    current_rotation = rotation;
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

bool SwerveController::is_paused() {
    return false;
}

SwerveController::~SwerveController()
{
}

