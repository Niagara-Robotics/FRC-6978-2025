#pragma once

#include "Task.h"
#include "FaultManager.h"
#include "ControlChannel.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <chrono>

#include <grpl/LaserCan.h>

namespace intake
{

enum class RotateCalibrationState {
    uncalibrated,
    calibrating,
    finished,
    failed
};

enum class IntakeAction {
    standby,
    pickup_coral,
    pickup_algae,
    eject_algae
};

enum class IntakeState {
    standby,
    algae_pickup,
    algae_locked,
    coral_vertical_a,
    coral_horizontal,
    coral_vertical_b
};

enum class IntakeClearanceLevel {
    none,
    lift_only,
    bay
};

class Intake  : public Task
{
private:
    //rotates the whole mechanism
    ctre::phoenix6::hardware::TalonFX rotate_motor = ctre::phoenix6::hardware::TalonFX(30, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration rotate_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(100)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(7.0_V) //TODO: increase limits
            .WithPeakReverseVoltage(-8.0_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(20_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(0.55_tps)
            .WithMotionMagicAcceleration(2.0_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(120).WithKI(0).WithKD(0.1)
            .WithKS(0.2).WithKV(2.7).WithKG(0.3).WithKA(0.4)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        );

    const units::angle::turn_t rotate_physical_stop_position = -0.37_tr;
    const units::angle::turn_t rotate_maximum_extent = 0.027_tr;
    const units::angle::turn_t rotate_lift_clearance_position = -0.265_tr;
    const units::angle::turn_t rotate_bay_clearance_position = -0.1_tr;

    const units::angle::turn_t algae_pickup_position = -0.11_tr;
    const units::angle::turn_t algae_hold_position = -0.205_tr;

    const units::angle::turn_t coral_vertical_a_position = -0.04_tr;
    const units::angle::turn_t coral_horizontal_position = 0.00_tr;
    const units::angle::turn_t coral_vertical_b_position = 0.026_tr;

    ctre::phoenix6::configs::SoftwareLimitSwitchConfigs rotate_softlimit_config = ctre::phoenix6::configs::SoftwareLimitSwitchConfigs()
        .WithForwardSoftLimitEnable(true)
        .WithForwardSoftLimitThreshold(rotate_maximum_extent)
        .WithReverseSoftLimitThreshold(rotate_physical_stop_position)
        .WithReverseSoftLimitEnable(true);

    //signals
    ctre::phoenix6::StatusSignal<units::temperature::celsius_t> rotate_temperature = rotate_motor.GetDeviceTemp();
    ctre::phoenix6::StatusSignal<units::current::ampere_t> rotate_current = rotate_motor.GetStatorCurrent();
    ctre::phoenix6::StatusSignal<units::angle::turn_t> rotate_motor_position = rotate_motor.GetPosition();

    ctre::phoenix6::controls::MotionMagicVoltage rotate_control = ctre::phoenix6::controls::MotionMagicVoltage(rotate_physical_stop_position);

    //two sets of rollers on the mechanism that manipulate the coral or algae
    ctre::phoenix6::hardware::TalonFX vertical_motor = ctre::phoenix6::hardware::TalonFX(31, "rio");
    ctre::phoenix6::hardware::TalonFX horizontal_motor = ctre::phoenix6::hardware::TalonFX(32, "rio");

    const ctre::phoenix6::configs::TalonFXConfiguration horizontal_vertical_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(10_A)
            .WithStatorCurrentLimitEnable(true)
        );

    const units::volt_t vertical_algae_pickup_voltage = 7.0_V;
    const units::volt_t vertical_algae_hold_voltage = 2.4_V;
    const units::volt_t vertical_algae_eject_voltage = -7.0_V;
    const units::volt_t horizontal_algae_pickup_voltage = 3.0_V;

    const units::volt_t vertical_coral_a_voltage = 6.5_V;
    const units::volt_t vertical_coral_b_voltage = 8.7_V;
    const units::volt_t horizontal_coral_voltage = 4.0_V;

    ctre::phoenix6::controls::VoltageOut vertical_control = ctre::phoenix6::controls::VoltageOut(0_V);
    ctre::phoenix6::controls::VoltageOut horizontal_control = ctre::phoenix6::controls::VoltageOut(0_V);

    //sensors
    ctre::phoenix6::hardware::CANcoder rotate_encoder = ctre::phoenix6::hardware::CANcoder(30, "rio");

    const ctre::phoenix6::configs::CANcoderConfiguration rotate_encoder_config = ctre::phoenix6::configs::CANcoderConfiguration()
        .WithMagnetSensor(ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithMagnetOffset(-0.186_tr - 0.25_tr));

    ctre::phoenix6::StatusSignal<units::angle::turn_t> rotate_encoder_position = rotate_encoder.GetAbsolutePosition();

    grpl::LaserCan intake_sensor = grpl::LaserCan(30);
    grpl::LaserCan staging_sensor = grpl::LaserCan(31);

    //in mm
    const uint16_t algae_lock_threshold = 100;
    const uint16_t coral_horizontal_threshold = 205;
    const uint16_t coral_a_threshold = 295;
    const uint16_t coral_hold_threshold = 120;

    //
    RotateCalibrationState rotate_calibration_state = RotateCalibrationState::uncalibrated; //and synced
    std::chrono::time_point<std::chrono::steady_clock> rotate_calibration_start;

    units::angle::turn_t rotate_target_position = rotate_physical_stop_position;

    IntakeState state = IntakeState::standby;
    bool coral_present = false;

    std::chrono::time_point<std::chrono::steady_clock> coral_horizontal_start;

    FaultManager fault_manager = FaultManager("intake");

    void configure_rotate_motor();
    void enable_rotate_softlimit();
    void end_rotate_calibration();

    void handle_pickup_coral();
    void handle_pickup_algae();
    void handle_eject_algae();

public:
    Intake(/* args */);
    ~Intake();

    controlchannel::ControlChannel<IntakeClearanceLevel> get_outta_the_way_channel = controlchannel::ControlChannel<IntakeClearanceLevel>(IntakeClearanceLevel::none);
    controlchannel::ControlChannel<IntakeAction> intake_action_channel = controlchannel::ControlChannel<IntakeAction>(IntakeAction::standby);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    IntakeClearanceLevel get_clearance_level();
    bool has_coral();
};
}