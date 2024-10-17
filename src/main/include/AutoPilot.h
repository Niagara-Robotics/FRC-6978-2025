#pragma once
#include "Task.h"
#include <frc/geometry/Pose2d.h>
#include "ControlChannel.h"
#include "SwerveController.h"
#include "Tracking.h"

enum class AutoPilotTwistMode {
    none,
    heading,
    face,
    pose
};

enum class AutoPilotTranslateMode {
    none,
    point
};

class AutoPilot: public Task
{
private:
    units::angle::radian_t heading_target = 90_deg;

    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;
    Tracking *tracking;

    units::angular_velocity::radians_per_second_t heading_proportional(units::angle::radian_t target, units::angle::radian_t current);

public:
    controlchannel::ControlChannel<AutoPilotTwistMode> twist_mode_channel = controlchannel::ControlChannel<AutoPilotTwistMode>(AutoPilotTwistMode::none);
    controlchannel::ControlChannel<units::angle::radian_t> heading_channel = controlchannel::ControlChannel<units::angle::radian_t>(0_rad);
    
    
    controlchannel::ControlChannel<frc::Pose2d> pose_channel = controlchannel::ControlChannel<frc::Pose2d>(frc::Pose2d());

    AutoPilot(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        Tracking *tracking):
        planar_handle(planar_handle), twist_handle(twist_handle), tracking(tracking) {}

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void set_rotation_mode(AutoPilotTwistMode mode);
    void set_hdg_target(units::angle::radian_t target);

    ~AutoPilot(){};
};