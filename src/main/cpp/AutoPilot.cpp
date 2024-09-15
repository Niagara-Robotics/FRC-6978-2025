#include "AutoPilot.h"
#include <frc/smartdashboard/SmartDashboard.h>

units::angular_velocity::radians_per_second_t heading_proportional(units::angle::radian_t target, units::angle::radian_t current) {
    units::angle::radian_t diff = (current - target);
    if(diff.value() > M_PI) diff-=M_PI*2_rad;
    if(diff.value() < -M_PI) diff+=M_PI*2_rad;

    units::angular_velocity::radians_per_second_t output = (-diff)* 5.8_rad_per_s/1_rad;
    if(output> 2.0_rad_per_s) output = 3.0_rad_per_s;
    if(output< -2.0_rad_per_s) output = -3.0_rad_per_s;
    if(fabs(output.value()) < 0.005) output = 0_rad_per_s;
    return output;
}

void AutoPilot::call() {
    switch (twist_mode_channel.get())
    {
    case heading: {//heading PID
        twist_handle.try_take_control();
        twist_handle.set(heading_proportional(heading_channel.get(), odometry->GetPose().Rotation().Radians()));
        break;
    }

    case face: {
        twist_handle.try_take_control();
        frc::Translation2d diff_translation = target_pose.Translation() - odometry->GetPose().Translation();
        double target = atan2(diff_translation.Y().value(), diff_translation.X().value());
        frc::SmartDashboard::PutNumber("autopilot/target_heading", target);
        twist_handle.set(heading_proportional(target * 1.0_rad, odometry->GetPose().Rotation().Radians()));
        break;
    }
    
    case none:
        twist_handle.set(0_rad_per_s);
        break;
    }
}

void AutoPilot::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(4000);
}

bool AutoPilot::is_paused() {
    return false;
}