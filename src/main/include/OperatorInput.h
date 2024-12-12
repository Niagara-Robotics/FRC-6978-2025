#pragma once

#include "Task.h"
#include "ControlChannel.h"
#include <frc/Joystick.h>
#include "SwerveController.h"
#include "AutoPilot.h"
#include "Tracking.h"
#include "NoteHandler.h"
#include "AutoShot.h"

class OperatorInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(1); //OPERATOR
    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;
    controlchannel::ControlHandle<AutoPilotTwistMode> ap_twist_mode_handle;
    controlchannel::ControlHandle<units::angle::radian_t> ap_heading_handle;

    controlchannel::ControlHandle<IntakeIndexingMode> index_mode_handle;
    controlchannel::ControlHandle<LauncherMode> launcher_mode_handle;
    controlchannel::ControlHandle<units::angular_velocity::turns_per_second_t> launcher_velocity_handle;

    controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle;

    controlchannel::ControlHandle<AutoShotMode> auto_shot_mode_handle;
    controlchannel::ControlHandle<units::angle::radian_t> auto_shot_offset_handle;

    Tracking *tracking;

    int last_pov = -1;
public:
    OperatorInput(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        controlchannel::ControlHandle<AutoPilotTwistMode> ap_twist_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> ap_heading_handle,
        controlchannel::ControlHandle<IntakeIndexingMode> index_mode_handle,
        controlchannel::ControlHandle<LauncherMode> launcher_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle,
        controlchannel::ControlHandle<AutoShotMode> auto_shot_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> auto_shot_offset_handle,
        controlchannel::ControlHandle<units::angular_velocity::turns_per_second_t> launcher_velocity_handle,
        Tracking *tracking):
        planar_handle(planar_handle), twist_handle(twist_handle), 
        ap_twist_mode_handle(ap_twist_mode_handle), ap_heading_handle(ap_heading_handle), tracking(tracking),
        index_mode_handle(index_mode_handle), launcher_mode_handle(launcher_mode_handle), launcher_tilt_handle(launcher_tilt_handle),
        auto_shot_mode_handle(auto_shot_mode_handle), auto_shot_offset_handle(auto_shot_offset_handle), launcher_velocity_handle(launcher_velocity_handle) {
            frc::SmartDashboard::PutNumber("launcher_test_angle", 0.7);
        }

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;
};