#include "SwerveController.h"
#include <iostream>

#include <frc/DigitalOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <thread>
#include <future>
#include <sched.h>
#include "ControlChannel.h"

#define DEAD_ZONE 0.15
#define xyMultiplier 2.0
#define wMultiplier 2.0

SwerveController::SwerveController(InputTest *input)
{
    this->input_system = input;
    new ControlChannel<int>(0);
    return;
}

void SwerveController::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(1800);
}

void SwerveController::call() {
    //sched_rr_get_interval(0, )
    //sched_setscheduler(0, SCHED_FIFO, new sched_param{.sched_priority = sched_get_priority_max(SCHED_FIFO)});
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    
    double x = -js.GetRawAxis(1);
    double y = -js.GetRawAxis(0);
    double omega = -js.GetRawAxis(2);

    x = (fabs(x) > DEAD_ZONE)? 
            ((x > 0)? 
                ((x-DEAD_ZONE)/(1-DEAD_ZONE)) :
                ((x+DEAD_ZONE)/(1-DEAD_ZONE))
            ) 
            : 0;

    y = (fabs(y) > DEAD_ZONE)? 
        ((y > 0)? 
            ((y-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((y+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;

    omega = (fabs(omega) > DEAD_ZONE)? 
        ((omega> 0)? 
            ((omega-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((omega+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;

    x *= xyMultiplier;
    y *= xyMultiplier;
    omega *= wMultiplier;

    target_chassis_speeds.vx = (units::velocity::meters_per_second_t)x;
    target_chassis_speeds.vy = (units::velocity::meters_per_second_t)y;
    target_chassis_speeds.omega = (units::angular_velocity::radians_per_second_t)omega;



    target_mutex.lock();
    
    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(target_chassis_speeds, input_system->get_last_rotation()), frc::Translation2d(0.0_m,0.0_m));
    target_mutex.unlock();

    std::chrono::duration<double, std::micro> calc_time = std::chrono::steady_clock::now() - start_time;

    double max_apply_time = 0;
    std::list<std::future<void>> futures = std::list<std::future<void>>();
    for (size_t i = 0; i < 4; i++)
    {
        //if(!enabled) break;
        //std::chrono::time_point<std::chrono::steady_clock> apply_start_time = std::chrono::steady_clock::now();
        //futures.push_back(std::async(&SwerveModule::apply, modules[i], states[i]));
        modules[i]->apply(states[i]);
        //modules[i]->test_couple();
        //std::chrono::duration<double, std::micro> apply_diff = std::chrono::steady_clock::now() - apply_start_time;
        //if(apply_diff.count() > max_apply_time) max_apply_time = apply_diff.count();
    }

    /*for(size_t i = 0; i < 4; i++) {
        futures.back().wait();
        futures.pop_back();
    }*/

    std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now() - start_time;
    
    //frc::SmartDashboard::PutNumber("swerve_controller_max_time_us", max_apply_time);
    frc::SmartDashboard::PutNumber("swerve_controller_total_time_us", diff.count());
    //frc::SmartDashboard::PutNumber("swerve_controller_calc_time_us", calc_time.count());

    //frc::SmartDashboard::PutNumber("swerve_controller_vx", target_chassis_speeds.vx.value());
    //frc::SmartDashboard::PutNumber("swerve_controller_vy", target_chassis_speeds.vy.value());

    for (int i = 0; i < 4; i++)
    {
        //frc::SmartDashboard::PutNumber("smc" + std::to_string(i) + "_status", modules[i]->get_state());
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

