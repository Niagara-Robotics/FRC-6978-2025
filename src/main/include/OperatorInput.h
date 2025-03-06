#pragma once

#include "Task.h"
#include "ControlChannel.h"
#include <frc/Joystick.h>
#include "SwerveController.h"
#include "Tracking.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Lift.h"
#include "Intake.h"

class OperatorInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(1); //OPERATOR
    controlchannel::ControlHandle<LateralSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;

    controlchannel::ControlHandle<intake::IntakeAction> intake_handle;

    controlchannel::ControlHandle<LiftMechanismState> lift_handle;
    controlchannel::ControlHandle<int> place_position_handle;

    Tracking *tracking;

    int last_pov = -1;

    FaultManager fault_manager = FaultManager("OperatorInput");
public:
    OperatorInput(
        controlchannel::ControlHandle<LateralSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<intake::IntakeAction> intake_handle,
        controlchannel::ControlHandle<LiftMechanismState> lift_handle,
        controlchannel::ControlHandle<int> place_position_handle,
        GlobalFaultManager *global_fm,
        Tracking *tracking):
        planar_handle(planar_handle), 
        twist_handle(twist_handle), 
        tracking(tracking), 
        intake_handle(intake_handle), 
        lift_handle(lift_handle), 
        place_position_handle(place_position_handle)
        {
            frc::SmartDashboard::PutNumber("launcher_test_angle", 0.7);
            global_fm->register_manager(&fault_manager);
        }

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;
};