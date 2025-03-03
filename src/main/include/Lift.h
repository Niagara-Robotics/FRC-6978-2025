#pragma once

#include "Task.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <grpl/LaserCan.h>

#include "ControlChannel.h"

#include "Intake.h"

enum class LiftMechanismState {
    park,
    pick,
    place
};

class Lift : public Task
{
private:

    ctre::phoenix6::hardware::TalonFX lift_motor = ctre::phoenix6::hardware::TalonFX(10, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration lift_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(50)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(1.0_V) //TODO: increase limits
            .WithPeakReverseVoltage(-1.0_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(20_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(0.30_tps)
            .WithMotionMagicAcceleration(1.8_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(35).WithKI(0).WithKD(0.1)
            .WithKS(0.2).WithKV(4.0).WithKG(0.25)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
        );

    const units::angle::turn_t max_lift_position = 3.0_tr;
    const units::angle::turn_t lift_park_position = 0.1_tr;

    const units::angle::turn_t lift_min_bay_position = 1.5_tr; //minimum position if the claw is in the bay
    const units::angle::turn_t lift_min_bay_traverse_position = 2.0_tr; //minimum position if the claw is 

    ctre::phoenix6::StatusSignal<units::angle::turn_t> lift_position = lift_motor.GetPosition();

    ctre::phoenix6::controls::MotionMagicVoltage lift_control = ctre::phoenix6::controls::MotionMagicVoltage(0_tr);

    ctre::phoenix6::hardware::TalonFX shoulder_motor = ctre::phoenix6::hardware::TalonFX(20, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration shoulder_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(100)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(1.0_V) //TODO: increase limits
            .WithPeakReverseVoltage(-2.0_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(12_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(0.35_tps)
            .WithMotionMagicAcceleration(1.8_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(120).WithKI(0).WithKD(0.1)
            .WithKS(0.2).WithKV(2.5).WithKG(0.25).WithKA(0.3)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
        );

    const units::angle::turn_t shoulder_park_position = 0_tr;
    const units::angle::turn_t shoulder_clear_position = 0.1_tr;
    const units::angle::turn_t shoulder_bay_exit_limit = 0.48_tr; //when the claw is in the bay
    const units::angle::turn_t shoulder_bay_enter_limit = 0.35_tr; //when the claw is NOT in the bay

    ctre::phoenix6::StatusSignal<units::angle::turn_t> shoulder_motor_position = shoulder_motor.GetPosition();

    ctre::phoenix6::controls::MotionMagicVoltage shoulder_control = ctre::phoenix6::controls::MotionMagicVoltage(-0.25_tr);

    ctre::phoenix6::hardware::CANcoder shoulder_encoder = ctre::phoenix6::hardware::CANcoder(20, "rio");

    ctre::phoenix6::configs::CANcoderConfiguration shoulder_encoder_configuration = ctre::phoenix6::configs::CANcoderConfiguration()
        .WithMagnetSensor(ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithMagnetOffset(0.1413_tr)
        );

    ctre::phoenix6::StatusSignal<units::angle::turn_t> shoulder_encoder_position = shoulder_motor.GetPosition();

    //state
    units::angle::turn_t target_shoulder_position;
    units::angle::turn_t target_lift_position = lift_park_position;

    LiftMechanismState current_mechanism_state = LiftMechanismState::park;
    LiftMechanismState target_mechanism_state = LiftMechanismState::park;

    controlchannel::ControlHandle<bool> get_out_handle;
    intake::Intake *intake;

    units::angle::turn_t filter_lift_position(units::angle::turn_t input);
    units::angle::turn_t filter_shoulder_position(units::angle::turn_t input);

public:

    Lift(intake::Intake *intake);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    ~Lift();
};
