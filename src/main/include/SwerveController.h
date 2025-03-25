#pragma once
#include "Task.h"
#include "SwerveModule.h"
#include <frc/DigitalOutput.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <list>
#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include "GyroInput.h"
#include "ControlChannel.h"

enum SwerveRequestType {
    full,
    full_robot_relative,
    rover,
    front_wheel_steer,
    rear_wheel_steer
};

class LateralSwerveRequest
{
public:
    units::velocity::meters_per_second_t x;
    units::velocity::meters_per_second_t y;
    units::angle::turn_t steer;

    SwerveRequestType request_type = SwerveRequestType::full;

    LateralSwerveRequest(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y): x(x), y(y) {}
    LateralSwerveRequest(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, SwerveRequestType type): x(x), y(y), request_type(type) {}
    LateralSwerveRequest(): x(0), y(0) {}
};

/**SwerveController manages kinematics and output to all swerve modules
 * SwerveManager owns the SwerveController
 * 
 */
class SwerveController: public Task
{
private:

    frc::Joystick js = frc::Joystick(0);

    bool enabled = false;

    ctre::phoenix6::configs::Slot0Configs drive_clc_gains = ctre::phoenix6::configs::Slot0Configs()
        .WithKP(0.09).WithKI(0).WithKD(0)
        .WithKS(0.14).WithKV(0.103).WithKA(0.0015);

    ctre::phoenix6::configs::Slot0Configs steer_clc_gains = ctre::phoenix6::configs::Slot0Configs()
        .WithKP(85.0).WithKI(0).WithKD(0)
        .WithKS(0.0).WithKV(0.95).WithKA(0.0005);

    ctre::phoenix6::configs::CurrentLimitsConfigs drive_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs()
        .WithStatorCurrentLimitEnable(true)
        .WithStatorCurrentLimit(45_A);

    ctre::phoenix6::configs::CurrentLimitsConfigs steer_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs()
        .WithStatorCurrentLimitEnable(true)
        .WithStatorCurrentLimit(50_A);

    ctre::phoenix6::configs::VoltageConfigs voltage_limits = ctre::phoenix6::configs::VoltageConfigs()
        .WithPeakForwardVoltage(11.8_V)
        .WithPeakReverseVoltage(-11.8_V);

    ctre::phoenix6::configs::CANcoderConfiguration steer_encoder_base_config;
    ctre::phoenix6::configs::TalonFXConfiguration drive_motor_base_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithSlot0(drive_clc_gains)
        .WithCurrentLimits(drive_current_limits)
        .WithVoltage(voltage_limits);

    ctre::phoenix6::configs::TalonFXConfiguration steer_motor_base_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithSlot0(steer_clc_gains)
        .WithCurrentLimits(steer_current_limits)
        .WithVoltage(voltage_limits);

    CommonSwerveModuleConfig common_config = CommonSwerveModuleConfig(
        steer_encoder_base_config,
        drive_motor_base_config,
        steer_motor_base_config,
        true,
        5.9, //drive ratio
        18.75, //steer ratio
        3.0, //couple ratio
        65_tr_per_s_sq, //steer acceleration
        5.0_tps, //steer velocity(cruise)
        2.0_in //wheel radius, 2 inches
    );

    wpi::array<frc::Translation2d, 4> module_positions = 
    {
        frc::Translation2d(-0.304_m, -0.304_m), //0
        frc::Translation2d(-0.304_m, 0.304_m), //1
        frc::Translation2d(0.304_m, -0.304_m), //2
        frc::Translation2d(0.304_m, 0.304_m) //3
    };

    //SMC1
    SwerveModuleConfig front_left_config = SwerveModuleConfig(
        &common_config,
        0, // steer encoder id
        0, // drive motor id
        1, // steer motor id
        -0.1067_tr + 0.5_tr, // steer offset
        false,  // drive motor invert
        -0.31,   // module position x
        -0.31    // module position y
    );

    //SMC2
    SwerveModuleConfig front_right_config = SwerveModuleConfig(
        &common_config,
        1, // steer encoder id
        2, // drive motor id
        3, // steer motor id
        0.444336_tr + 0.5_tr, // steer offset
        true,  // drive motor invert
        0.31,   // module position x
        -0.31   // module position y
    );

    //SMC3
    SwerveModuleConfig back_left_config = SwerveModuleConfig(
        &common_config,
        2, // steer encoder id
        4, // drive motor id
        5, // steer motor id
        0.3276_tr + 0.5_tr, // steer offset
        false, // drive motor invert
        -0.31, // module position x
        0.31   // module position y
    );

    //SMC4
    SwerveModuleConfig back_right_config = SwerveModuleConfig(
        &common_config,
        3, // steer encoder id
        6, // drive motor id
        7, // steer motor id
        0.0703_tr + 0.5_tr,// steer offset
        true, // drive motor invert
        0.31, // module position x
        0.31  // module position y
    );

    wpi::array<SwerveModule*, 4> modules = 
    {
        new SwerveModule(front_left_config, "SMC1"),
        new SwerveModule(front_right_config, "SMC2"),
        new SwerveModule(back_left_config, "SMC3"),
        new SwerveModule(back_right_config, "SMC4")
    };

    wpi::array<frc::SwerveModulePosition, 4> last_reported_positions =
    {
        frc::SwerveModulePosition{0_m, frc::Rotation2d()},
        frc::SwerveModulePosition{0_m, frc::Rotation2d()},
        frc::SwerveModulePosition{0_m, frc::Rotation2d()},
        frc::SwerveModulePosition{0_m, frc::Rotation2d()}
    };

    wpi::array<frc::SwerveModuleState, 4> target_states = wpi::array<frc::SwerveModuleState, 4>(wpi::empty_array);

    frc::SwerveDriveKinematics<4> kinematics = frc::SwerveDriveKinematics<4>(module_positions);

    frc::ChassisSpeeds target_chassis_speeds;
    bool field_relative_drive;

    std::mutex target_mutex;

    frc::Rotation2d current_rotation;

    nt::StructArrayPublisher<frc::SwerveModulePosition> module_positions_publisher;

    //ctre::phoenix6::hardware::TalonFX fake_talon = ctre::phoenix6::hardware::TalonFX(26);

public:
    controlchannel::ControlChannel<LateralSwerveRequest> planar_velocity_channel = controlchannel::ControlChannel(LateralSwerveRequest(0_mps, 0_mps));
    controlchannel::ControlChannel<units::angular_velocity::radians_per_second_t> twist_velocity_channel = controlchannel::ControlChannel(0_rad_per_s);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    void set_chassis_rotation(frc::Rotation2d rotation);

    wpi::array<frc::SwerveModulePosition, 4> fetch_module_positions();
    std::vector<frc::Translation2d> fetch_module_offsets();
    frc::ChassisSpeeds get_chassis_speeds();

    frc::SwerveDriveKinematics<4> get_kinematics();

    SwerveController(GlobalFaultManager *global_fm);
    ~SwerveController();
};