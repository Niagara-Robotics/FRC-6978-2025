#pragma once
#include "Task.h"
#include <frc/geometry/Pose2d.h>
#include "ControlChannel.h"
#include "SwerveController.h"

typedef enum AutoPilotTwistMode {
    none,
    heading,
    face
};

class AutoPilot: public Task
{
private:
    units::angle::radian_t heading_target = 90_deg;
    frc::Pose2d target_pose;

    controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle;
    controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle;

    AutoPilotTwistMode current_twist_mode = none;

    frc::SwerveDriveOdometry<4> *odometry;

public:
    controlchannel::ControlChannel<AutoPilotTwistMode> twist_mode_channel = controlchannel::ControlChannel<AutoPilotTwistMode>(none);
    controlchannel::ControlChannel<units::angle::radian_t> heading_channel = controlchannel::ControlChannel<units::angle::radian_t>(0_rad);

    AutoPilot(
        controlchannel::ControlHandle<PlanarSwerveRequest> planar_handle, 
        controlchannel::ControlHandle<units::angular_velocity::radians_per_second_t> twist_handle,
        frc::SwerveDriveOdometry<4> *odometry):
        planar_handle(planar_handle), twist_handle(twist_handle), odometry(odometry) {}

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call() override;
    bool is_paused() override;

    void set_rotation_mode(AutoPilotTwistMode mode);
    void set_hdg_target(units::angle::radian_t target);

    ~AutoPilot(){};
};