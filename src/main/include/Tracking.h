#pragma once

#include "Task.h"
#include "SwerveController.h"
#include "GyroInput.h"
#include <networktables/NetworkTable.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <frc/estimator/KalmanFilter.h>

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
#define PS_OCI_SUBCLASS_LATENCY 0x06

#define PS_OTI_INTEGER 0x03
#define PS_OTI_DOUBLE 0x04
#define PS_OTI_2D_ROTATION 0x11
#define PS_OTI_3D_POSITION 0x12
#define PS_OTI_3D_EULER 0x13

#define TRACKING_BUFFER_CAP 256

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

class TrackingFrame {
public:
    frc::Rotation2d rotation_estimate;
    frc::Rotation2d gyro_rotation;

    wpi::array<frc::SwerveModulePosition, 4> module_positions;

    frc::Pose2d estimated_pose;

    std::chrono::time_point<std::chrono::steady_clock> observation_time;

    TrackingFrame(frc::Rotation2d rotation_estimate, frc::Rotation2d gyro_rotation, frc::Pose2d estimated_pose, wpi::array<frc::SwerveModulePosition, 4> module_positions): rotation_estimate(rotation_estimate), gyro_rotation(gyro_rotation), estimated_pose(estimated_pose), module_positions(module_positions) {
        observation_time = std::chrono::steady_clock::now();
    }
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
    nt::StructPublisher<frc::Pose2d> camera_pose_publisher;
    nt::StructPublisher<frc::Pose2d> tag_pose_publisher;

    frc::Pose2d field_map[22] = {
        frc::Pose2d(frc::Translation2d(657.37_in, 25.8_in), frc::Rotation2d(126_deg)),
        frc::Pose2d(frc::Translation2d(657.37_in, 291.2_in), frc::Rotation2d(234_deg)),
        frc::Pose2d(frc::Translation2d(455_in, 317.15_in), frc::Rotation2d(270_deg)),
        frc::Pose2d(frc::Translation2d(365.2_in, 241.64_in), frc::Rotation2d(0_deg)),
        frc::Pose2d(frc::Translation2d(365.2_in, 75.39_in), frc::Rotation2d(0_deg)),
        frc::Pose2d(frc::Translation2d(530.49_in, 130.17_in), frc::Rotation2d(300_deg)), //6
        frc::Pose2d(frc::Translation2d(546.87_in, 158.5_in), frc::Rotation2d(0_deg)), //7
        frc::Pose2d(frc::Translation2d(530.49_in, 186.83_in), frc::Rotation2d(60_deg)), // 8
        frc::Pose2d(frc::Translation2d(497.77_in, 186.83_in), frc::Rotation2d(120_deg)),
        frc::Pose2d(frc::Translation2d(481.39_in, 158.5_in), frc::Rotation2d(180_deg)), //10
        frc::Pose2d(frc::Translation2d(497.77_in, 130.17_in), frc::Rotation2d(240_deg)),
        frc::Pose2d(frc::Translation2d(33.51_in, 25.8_in), frc::Rotation2d(54_deg)), //12
        frc::Pose2d(frc::Translation2d(33.51_in, 291.2_in), frc::Rotation2d(306_deg)),
        frc::Pose2d(frc::Translation2d(325.68_in, 241.64_in), frc::Rotation2d(180_deg)), //14
        frc::Pose2d(frc::Translation2d(325.68_in, 75.39_in), frc::Rotation2d(180_deg)),
        frc::Pose2d(frc::Translation2d(235.73_in, -0.15_in), frc::Rotation2d(90_deg)), //16
        frc::Pose2d(frc::Translation2d(160.39_in, 130.17_in), frc::Rotation2d(240_deg)),
        frc::Pose2d(frc::Translation2d(144.0_in, 158.5_in), frc::Rotation2d(180_deg)), //18
        frc::Pose2d(frc::Translation2d(160.39_in, 186.83_in), frc::Rotation2d(120_deg)),
        frc::Pose2d(frc::Translation2d(193.1_in, 186.83_in), frc::Rotation2d(60_deg)), //20
        frc::Pose2d(frc::Translation2d(209.49_in, 158.5_in), frc::Rotation2d(0_deg)),
        frc::Pose2d(frc::Translation2d(192.1_in, 130.17_in), frc::Rotation2d(300_deg)) //22
    };

    //frc::KalmanFilter filter = frc::KalmanFilter();
    std::list<TrackingFrame> tracking_buffer = std::list<TrackingFrame>();

    SpeakerReport last_speaker_report;

    std::chrono::time_point<std::chrono::steady_clock> last_camera_pose_update;
    void handle_packet(char buf[256]);

    std::chrono::time_point<std::chrono::steady_clock> last_sent_heartbeat;
    std::chrono::time_point<std::chrono::steady_clock> last_received_heartbeat;
    std::chrono::milliseconds last_network_latency;

    std::shared_ptr<nt::NetworkTable> ui_table = nt::NetworkTableInstance(nt::GetDefaultInstance()).GetTable("tracking");

    void send_heartbeat();

    void drive_robot_relative(frc::ChassisSpeeds speeds);

    void push_camera_update(bool rotation, frc::Pose2d pose, std::chrono::steady_clock::time_point exposure_timestamp);
public:
    Tracking(SwerveController *swerve_controller);

    controlchannel::ControlChannel<bool> mask_vision_channel = controlchannel::ControlChannel<bool>(true);

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
