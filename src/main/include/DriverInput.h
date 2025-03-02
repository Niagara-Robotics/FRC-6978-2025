#pragma once

#include "Task.h"
#include "ControlChannel.h"
#include <frc/Joystick.h>
#include "SwerveController.h"
#include "Tracking.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "FaultManager.h"
#include "Intake.h"


class DriverInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(0); //DRIVER
    controlchannel::ControlHandle<LateralSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;

    controlchannel::ControlHandle<intake::IntakeAction> intake_handle;

    Tracking *tracking;

    bool last_rrel_button = false;
    bool robot_relative = false;

    FaultManager fault_manager = FaultManager("DriverInput");
public:
    DriverInput(
        controlchannel::ControlHandle<LateralSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<intake::IntakeAction> intake_handle,
        GlobalFaultManager *global_fm,
        Tracking *tracking):
        planar_handle(planar_handle), twist_handle(twist_handle), intake_handle(intake_handle)
        {
            frc::SmartDashboard::PutNumber("launcher_test_angle", 0.7);
            global_fm->register_manager(&fault_manager);
        }

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;
};