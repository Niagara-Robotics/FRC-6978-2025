#include "SwerveController.h"
#include <iostream>

#include <frc/DigitalOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <thread>
#include <future>
#include <sched.h>
#include "ControlChannel.h"

SwerveController::SwerveController(GyroInput *input): input_system(input)
{
    this->input_system = input;
    return;
}

void SwerveController::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(1800);
}

void SwerveController::call() {
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

    PlanarSwerveRequest planar_request = planar_velocity_channel.get();

    target_chassis_speeds.vx = planar_request.x;
    target_chassis_speeds.vy = planar_request.y;
    target_chassis_speeds.omega = twist_velocity_channel.get();
    
    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(target_chassis_speeds, input_system->get_last_rotation()), frc::Translation2d(0.0_m,0.0_m));

    double max_apply_time = 0;
    std::list<std::future<void>> futures = std::list<std::future<void>>();
    for (size_t i = 0; i < 4; i++)
    {
        modules[i]->apply(states[i]);
        last_reported_positions[i] = modules[i]->get_position();
    }

    odometry.Update(input_system->get_last_rotation(), last_reported_positions);

    std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now() - start_time;

    frc::SmartDashboard::PutNumber("swerve_controller_total_time_us", diff.count());

    for (int i = 0; i < 4; i++)
    {
        frc::SmartDashboard::PutNumber("smc" + std::to_string(i) + "_status", modules[i]->get_state());
    }
}

void SwerveController::notify_enabled(bool enabled) {
    this->enabled = enabled;
}

bool SwerveController::is_paused() {
    return false;
}

SwerveController::~SwerveController()
{
}

