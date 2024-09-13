#include "SwerveModule.h"

CommonSwerveModuleConfig::CommonSwerveModuleConfig(
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
)
{
    this->steer_encoder_base_config = steer_encoder_base_config;
    this->drive_motor_base_config = drive_motor_base_config;
    this->steer_motor_base_config = steer_motor_base_config;

    this->steer_inverted = steer_inverted;
    
    this->drive_ratio = drive_ratio;
    this->steer_ratio = steer_ratio;
    this->couple_ratio = couple_ratio;

    this->steer_acceleration = steer_acceleration;
    this->steer_velocity = steer_velocity;

    this->wheel_radius = wheel_radius;
}

SwerveModuleConfig::SwerveModuleConfig(
    CommonSwerveModuleConfig *common_config,
    int steer_encoder_id,
    int drive_motor_id,
    int steer_motor_id,
    double steering_offset,
    bool drive_inverted,
    double position_X,
    double position_Y
)
{
    this->steer_encoder_id = steer_encoder_id;
    this->drive_motor_id = drive_motor_id;
    this->steer_motor_id = steer_motor_id;

    this->steering_offset = steering_offset;
    this->drive_inverted = drive_inverted;
    this->steer_inverted = common_config->steer_inverted;

    this->position_X = position_X;
    this->position_Y = position_Y;

    this->steer_encoder_base_config = common_config->steer_encoder_base_config;
    this->drive_motor_base_config = common_config->drive_motor_base_config;
    this->steer_motor_base_config = common_config->steer_motor_base_config;

    this->drive_ratio = common_config->drive_ratio;
    this->steer_ratio = common_config->steer_ratio;
    this->couple_ratio = common_config->couple_ratio;

    this->steer_acceleration = common_config->steer_acceleration;
    this->steer_velocity = common_config->steer_velocity;

    this->wheel_radius = common_config->wheel_radius;
}

SwerveModuleConfig::SwerveModuleConfig()
{

}