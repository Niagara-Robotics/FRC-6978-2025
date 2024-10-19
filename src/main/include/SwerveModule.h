#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <list>

class CommonSwerveModuleConfig
{
public:
    ctre::phoenix6::configs::CANcoderConfiguration steer_encoder_base_config;
    ctre::phoenix6::configs::TalonFXConfiguration drive_motor_base_config;
    ctre::phoenix6::configs::TalonFXConfiguration steer_motor_base_config;

    bool steer_inverted; //only affects motor

    double drive_ratio; //rotor:wheel
    double steer_ratio; //rotor:azimuth rotation
    double couple_ratio; //drive rotor:azimuth rotation

    double steer_acceleration, steer_velocity;

    units::length::meter_t wheel_radius;

    CommonSwerveModuleConfig(
        ctre::phoenix6::configs::CANcoderConfiguration steer_encoder_base_config,
        ctre::phoenix6::configs::TalonFXConfiguration drive_motor_base_config,
        ctre::phoenix6::configs::TalonFXConfiguration steer_motor_base_config,
        bool steer_inverted,
        double drive_ratio,
        double steer_ratio,
        double couple_ratio,
        double steer_acceleration,
        double steer_velocity,
        
        units::length::meter_t wheel_radius
    );
};

class SwerveModuleConfig
{
public:
    int steer_encoder_id;
    int drive_motor_id;
    int steer_motor_id;

    ctre::phoenix6::configs::CANcoderConfiguration steer_encoder_base_config;
    ctre::phoenix6::configs::TalonFXConfiguration drive_motor_base_config;
    ctre::phoenix6::configs::TalonFXConfiguration steer_motor_base_config;

    bool steer_inverted; //only affects motor
    bool drive_inverted;

    double drive_ratio; //rotor:wheel
    double steer_ratio; //rotor:azimuth rotation
    double couple_ratio; //drive rotor:azimuth rotation

    double steer_acceleration;
    double steer_velocity;

    units::length::meter_t wheel_radius;

    double steering_offset;

    double position_X;
    double position_Y;

    SwerveModuleConfig(
        CommonSwerveModuleConfig *common_config,
        int steer_encoder_id,
        int drive_motor_id,
        int steer_motor_id,
        double steering_offset,
        bool drive_inverted,
        double position_X,
        double position_y
    );

    SwerveModuleConfig();
};

class SwerveModule
{
private:
    ctre::phoenix6::hardware::CANcoder *steering_encoder;
    ctre::phoenix6::hardware::TalonFX *drive_motor;
    ctre::phoenix6::hardware::TalonFX *steer_motor;

    ctre::phoenix6::StatusSignal<units::angle::turn_t> *steering_position;
    ctre::phoenix6::StatusSignal<units::angle::turn_t> *relative_steering_position;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *steering_velocity;

    ctre::phoenix6::StatusSignal<units::angle::turn_t> *drive_position;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *drive_velocity;

    ctre::phoenix6::controls::MotionMagicVoltage *steer_control;
    ctre::phoenix6::controls::VelocityVoltage *drive_control;

    units::length::meter_t wheel_circumference;

    SwerveModuleConfig config;

    units::angle::turn_t last_steering_relative_position;
    units::angle::turn_t drive_position_correction;

    void log(std::string message);

    int state;
    std::list<int> faults;

    std::string id;

public:
    SwerveModule(SwerveModuleConfig config, std::string id);
    void apply(frc::SwerveModuleState state);
    void idle();
    void test_couple();
    frc::SwerveModulePosition get_position();
    frc::SwerveModuleState get_module_state();
    int get_state();
    ~SwerveModule();
};

#define SMC_STATE_STOPPED 0
#define SMC_STATE_DEGRADED 1
#define SMC_STATE_TESTING 2
#define SMC_STATE_INITIALIZING 3
#define SMC_STATE_NORMAL 4
#define SMC_STATE_IDLE 5

#define SMC_FAULT_COMMUNICATION 0
#define SMC_FAULT_STEER_RESPONSE 1
#define SMC_FAULT_STEER_TEMPERATURE 2
#define SMC_FAULT_DRIVE_TEMPERATURE 3
