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
#include <frc/DriverStation.h>

#define POSE_STREAMER_PORT 6000

class SpeakerReport {
public:
    std::chrono::time_point<std::chrono::steady_clock> observation_time;
    units::length::meter_t distance;
    units::angle::radian_t heading;

    SpeakerReport() {};

    SpeakerReport(std::chrono::time_point<std::chrono::steady_clock> observation_time,
    units::length::meter_t distance,
    units::angle::radian_t heading): observation_time(observation_time), distance(distance), heading(heading) {};
};

class Tracking : public Task
{
private:
    SwerveController *swerve_controller;

    frc::Rotation2d recent_gyro_pose;
    frc::Rotation2d previous_gyro_pose;
    frc::Rotation2d gyro_offset;
    frc::Rotation2d gyro_rate;
    frc::Rotation2d current_rotation_estimate;
    studica::AHRS *mxp;


    bool gyro_degraded = true;
    bool camera_orientation_enabled = true;

    void update_gyro();

    int sockfd;

    double last_mxp_timestamp;
    std::chrono::time_point<std::chrono::steady_clock> mxp_update_timestamp;

    frc::SwerveDriveOdometry<4> *swerve_odometry;

    nt::StructPublisher<frc::Pose2d> odometry_pose_publisher;

    SpeakerReport last_speaker_report;

    void handle_packet(char buf[256]);

    void drive_robot_relative(frc::ChassisSpeeds speeds);
public:
    Tracking(SwerveController *swerve_controller);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void set_gyro_angle(frc::Rotation2d target_rotation);

    frc::ChassisSpeeds get_chassis_speeds();

    frc::Pose2d get_pose();
    void reset();
    void reset_pose(frc::Pose2d pose);

    void set_camera_orientation_enabled(bool enabled);

    SpeakerReport get_speaker_pose();

    ~Tracking();
};
