#pragma once

#include "Task.h"
#include "SwerveController.h"
#include "GyroInput.h"
#include <networktables/NetworkTable.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define POSE_STREAMER_PORT 6000

class Tracking : public Task
{
private:
    SwerveController *swerve_controller;

    frc::Rotation2d recent_gyro_pose;
    frc::Rotation2d previous_gyro_pose;
    frc::Rotation2d gyro_offset;
    frc::Rotation2d gyro_rate;
    frc::Rotation2d current_rotation_estimate;
    AHRS *mxp;

    bool gyro_degraded = false;

    void update_gyro();

    int sockfd;

    double last_mxp_update_count;
    std::chrono::time_point<std::chrono::steady_clock> mxp_update_timestamp;

    frc::SwerveDriveOdometry<4> *swerve_odometry;

    nt::StructPublisher<frc::Pose2d> odometry_pose_publisher;

    void handle_packet(char buf[256]);
public:
    Tracking(SwerveController *swerve_controller);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void set_gyro_angle(frc::Rotation2d target_rotation);

    frc::Pose2d get_pose();
    void reset();

    ~Tracking();
};
