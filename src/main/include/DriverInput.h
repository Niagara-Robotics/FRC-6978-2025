#pragma once

#include "Task.h"
#include "ControlChannel.h"
#include <frc/Joystick.h>
#include "SwerveController.h"

class DriverInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(0);
    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;
public:
    DriverInput(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle):
        planar_handle(planar_handle), twist_handle(twist_handle) {}

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call() override;
    bool is_paused() override;
};