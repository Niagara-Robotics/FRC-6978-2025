#pragma once

#include "Task.h"
#include "ControlChannel.h"
#include <frc/Joystick.h>
#include "SwerveController.h"
#include "Tracking.h"
#include <frc/smartdashboard/SmartDashboard.h>


class DriverInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(0); //DRIVER
    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;

    Tracking *tracking;

    bool last_rrel_button = false;
    bool robot_relative = false;
public:
    DriverInput(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,

        Tracking *tracking):
        planar_handle(planar_handle), twist_handle(twist_handle) {
            frc::SmartDashboard::PutNumber("launcher_test_angle", 0.7);
        }

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;
};