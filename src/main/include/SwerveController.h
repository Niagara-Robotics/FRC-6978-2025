#pragma once
#include "Task.h"
#include "SwerveModule.h"
#include <frc/DigitalOutput.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <list>
#include <frc/Joystick.h>
#include "InputTest.h"
#include "ControlChannel.h"

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
        .WithKP(0.018).WithKI(0).WithKD(0)
        .WithKS(0.015).WithKV(0.12).WithKA(0.004);

    ctre::phoenix6::configs::Slot0Configs steer_clc_gains = ctre::phoenix6::configs::Slot0Configs()
        .WithKP(82.0).WithKI(0).WithKD(0)
        .WithKS(0.12).WithKV(0).WithKA(0.00);

    ctre::phoenix6::configs::CurrentLimitsConfigs drive_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs()
        .WithStatorCurrentLimitEnable(true)
        .WithStatorCurrentLimit(40);

    ctre::phoenix6::configs::CurrentLimitsConfigs steer_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs()
        .WithStatorCurrentLimitEnable(true)
        .WithStatorCurrentLimit(40);

    ctre::phoenix6::configs::VoltageConfigs voltage_limits = ctre::phoenix6::configs::VoltageConfigs()
        .WithPeakForwardVoltage(8.5)
        .WithPeakReverseVoltage(-8.5);

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
        65, //steer acceleration
        5.2, //steer velocity(cruise)
        2_in //wheel radius, 2 inches
    );

    wpi::array<frc::Translation2d, 4> module_positions = 
    {
        frc::Translation2d(0.31_m, 0.31_m),
        frc::Translation2d(0.31_m, -0.31_m),
        frc::Translation2d(-0.31_m, 0.31_m),
        frc::Translation2d(-0.31_m, -0.31_m)
    };

    //SMC1
    SwerveModuleConfig front_left_config = SwerveModuleConfig(
        &common_config,
        0, // steer encoder id
        0, // drive motor id
        1, // steer motor id
        0.0676, // steer offset
        false,  // drive motor invert
        0.31,   // module position x
        0.31    // module position y
    );

    //SMC2
    SwerveModuleConfig front_right_config = SwerveModuleConfig(
        &common_config,
        1, // steer encoder id
        2, // drive motor id
        3, // steer motor id
        0.3298, // steer offset
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
        0.447, // steer offset
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
        -0.106,// steer offset
        true, // drive motor invert
        -0.31, // module position x
        -0.31  // module position y
    );

    wpi::array<SwerveModule*, 4> modules = 
    {
        new SwerveModule(front_left_config, "SMC1"),
        new SwerveModule(front_right_config, "SMC2"),
        new SwerveModule(back_left_config, "SMC3"),
        new SwerveModule(back_right_config, "SMC4")
    };

    frc::SwerveDriveKinematics<4> kinematics = frc::SwerveDriveKinematics<4>(module_positions);
    //frc::SwerveDriveOdometry<4> odometry = frc::SwerveDriveOdometry<4>(kinematics, module_positions);

    frc::ChassisSpeeds target_chassis_speeds;
    bool field_relative_drive;

    std::mutex target_mutex;

    InputTest *input_system;

    controlchannel::ControlChannel<frc::ChassisSpeeds> planar_velocity_channel = controlchannel::ControlChannel(frc::ChassisSpeeds());

public:
    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call() override;
    bool is_paused() override;

    //void register_module();

    void notify_enabled(bool enabled);

    SwerveController(InputTest *input);
    ~SwerveController();
};