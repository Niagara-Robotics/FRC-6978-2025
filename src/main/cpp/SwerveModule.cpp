#include "SwerveModule.h"

#include <cmath>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

//NOTE: this is specifically optimized for coax swerve devices! set config.couple_ratio to 0 to disable coax behavior

SwerveModule::SwerveModule(SwerveModuleConfig module_config, std::string id)
{   
    this->state = SMC_STATE_INITIALIZING;

    this->config = module_config;

    //setup steering encoder
    config.steer_encoder_base_config.MagnetSensor.MagnetOffset = config.steering_offset;
    
    steering_encoder = new ctre::phoenix6::hardware::CANcoder(config.steer_encoder_id, "drive");
    steering_encoder->GetConfigurator().Apply(config.steer_encoder_base_config);

    steering_position = &steering_encoder->GetAbsolutePosition();
    steering_position->SetUpdateFrequency(1000_Hz, 2_s); //maximum update frequency

    relative_steering_position = &steering_encoder->GetPosition();
    relative_steering_position->SetUpdateFrequency(1000_Hz, 2_s);

    last_steering_relative_position = relative_steering_position->GetValue();

    steering_velocity = &steering_encoder->GetVelocity();
    steering_velocity->SetUpdateFrequency(1000_Hz, 2_s);

    //setup drive motor
    config.drive_motor_base_config.MotorOutput.Inverted = config.drive_inverted?  signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;

    drive_motor = new hardware::TalonFX(config.drive_motor_id, "drive");
    drive_motor->GetConfigurator().Apply(config.drive_motor_base_config);

    drive_position = &drive_motor->GetPosition();
    drive_position->SetUpdateFrequency(1_kHz, 2_s);

    drive_velocity = &drive_motor->GetVelocity();
    drive_velocity->SetUpdateFrequency(100_Hz, 2_s);

    drive_control = new controls::VelocityVoltage(0_tps);
    drive_control->UpdateFreqHz = 1000_Hz; //synchronous request

    drive_motor->SetControl(*drive_control);

    //setup steering motor

    config.steer_motor_base_config.Feedback.FeedbackRemoteSensorID = config.steer_encoder_id;
    config.steer_motor_base_config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    config.steer_motor_base_config.Feedback.RotorToSensorRatio = config.steer_ratio;
    config.steer_motor_base_config.ClosedLoopGeneral.ContinuousWrap = true;

    if(config.steer_inverted) {
        config.steer_motor_base_config.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    }
    config.steer_motor_base_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    config.steer_motor_base_config.MotionMagic.MotionMagicAcceleration = config.steer_acceleration;
    config.steer_motor_base_config.MotionMagic.MotionMagicCruiseVelocity = config.steer_velocity;

    steer_motor = new hardware::TalonFX(config.steer_motor_id, "drive");
    steer_motor->GetConfigurator().Apply(config.steer_motor_base_config);

    steer_control = new controls::MotionMagicVoltage(0_tr);
    steer_control->UpdateFreqHz = 1000_Hz;

    steer_motor->SetControl(*steer_control);

    wheel_circumference = config.wheel_radius * M_PI * 2;
    std::cout << "wheel circumference " << wheel_circumference.value() << std::endl;

    this->state = SMC_STATE_IDLE;
    this->id = id;
}

void SwerveModule::apply(frc::SwerveModuleState target_state) 
{
    this->state = SMC_STATE_NORMAL;
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

    if(BaseStatusSignal::RefreshAll(*steering_position, *steering_velocity) != 0) {
        this->state = 10;
    }

    target_state = frc::SwerveModuleState::Optimize(target_state, frc::Rotation2d(steering_position->GetValue()));
    //set the steering output 
    steer_control->Position = target_state.angle.Radians().convert<units::angle::turn>();
    
    std::chrono::duration<double, std::micro> steer_calc_time = std::chrono::steady_clock::now() - start_time;
    start_time = std::chrono::steady_clock::now();

    if (steer_motor->SetControl(*steer_control) != 0) {
        this->state = 20;
    }

    std::chrono::duration<double, std::micro> steer_apply_time = std::chrono::steady_clock::now() - start_time;
    start_time = std::chrono::steady_clock::now();

    //cosine compensation for drive velocity
    units::angular_velocity::turns_per_second_t drive_velocity = target_state.speed / (wheel_circumference / 1_tr) * config.drive_ratio;
    drive_velocity *= std::cos((steering_position->GetValue().convert<units::angle::radian>() - target_state.angle.Radians()).value());

    drive_velocity += steering_velocity->GetValue() * ((config.drive_inverted)? -config.couple_ratio : config.couple_ratio);

    drive_control->Velocity = drive_velocity;

    std::chrono::duration<double, std::micro> drive_calc_time = std::chrono::steady_clock::now() - start_time;
    start_time = std::chrono::steady_clock::now();
    
    if (drive_motor->SetControl(*drive_control) != 0) {
        this->state = 30;
    }

    std::chrono::duration<double, std::micro> drive_apply_time = std::chrono::steady_clock::now() - start_time;
    start_time = std::chrono::steady_clock::now();

    //frc::SmartDashboard::PutNumber(id+"/steer_calc_time", steer_calc_time.count());
    //frc::SmartDashboard::PutNumber(id+"/steer_apply_time", steer_apply_time.count());
    //frc::SmartDashboard::PutNumber(id+"/drive_calc_time", drive_calc_time.count());
    //frc::SmartDashboard::PutNumber(id+"/drive_apply_time", drive_apply_time.count());
}

frc::SwerveModulePosition SwerveModule::get_position() {
    if(BaseStatusSignal::RefreshAll(*steering_position, *drive_position, *relative_steering_position) != 0) {
        this->state = 10;
    }

    //correct the drive position
    drive_position_correction += ((relative_steering_position->GetValue() - last_steering_relative_position) * ((config.drive_inverted)? -config.couple_ratio : config.couple_ratio));

    return frc::SwerveModulePosition{(drive_position->GetValue() + drive_position_correction) / config.drive_ratio * (wheel_circumference / 1_tr), frc::Rotation2d(steering_position->GetValue())};
}

void SwerveModule::test_couple() {
    if(BaseStatusSignal::RefreshAll(*steering_position, *steering_velocity) != 0) {
        this->state = SMC_STATE_DEGRADED;
    }
    controls::VoltageOut steer_static_control(2.5_V);

    steer_motor->SetControl(steer_static_control);

    units::angular_velocity::turns_per_second_t drive_velocity;
    drive_velocity += steering_velocity->GetValue() * ((config.drive_inverted)? -config.couple_ratio : config.couple_ratio);
    drive_control->Velocity = drive_velocity;
    
    if (drive_motor->SetControl(*drive_control) != 0) {
        this->state = SMC_STATE_DEGRADED;
    }
}

int SwerveModule::get_state() {
    return state;
}

SwerveModule::~SwerveModule()
{
}