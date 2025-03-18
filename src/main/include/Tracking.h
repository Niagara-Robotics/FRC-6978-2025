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

#define PS_OCI_CLASS_TRACKING 0x10
#define PS_OCI_SUBCLASS_ROBOT_CAMERA_POSE 0x00
#define PS_OCI_SUBCLASS_ROBOT_GYRO_POSE 0x01
#define PS_OCI_SUBCLASS_TAG_DISTANCE 0x03
#define PS_OCI_SUBCLASS_TAG_HEADING 0x05
#define PS_OCI_SUBCLASS_TAG_ID 0x04

#define PS_OTI_INTEGER 0x03
#define PS_OTI_DOUBLE 0x04
#define PS_OTI_2D_ROTATION 0x11
#define PS_OTI_3D_POSITION 0x12

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

    std::chrono::time_point<std::chrono::steady_clock> last_gyro_camera_sync;

    void update_gyro();
    void update_orientation_estimate();

    const char *vc_host = "10.69.78.3";
    int sockfd;

    double last_mxp_timestamp;
    std::chrono::time_point<std::chrono::steady_clock> mxp_last_update_local_timestamp;

    frc::SwerveDriveOdometry<4> *swerve_odometry;

    nt::StructPublisher<frc::Pose2d> odometry_pose_publisher;

    SpeakerReport last_speaker_report;

    std::chrono::time_point<std::chrono::steady_clock> last_camera_pose_update;
    void handle_packet(char buf[256]);

    std::chrono::time_point<std::chrono::steady_clock> last_sent_heartbeat;
    std::chrono::time_point<std::chrono::steady_clock> last_received_heartbeat;
    void send_heartbeat();

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
