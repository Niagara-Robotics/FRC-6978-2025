#pragma once

#include "Task.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <networktables/NetworkTable.h>

#include <grpl/LaserCan.h>

#include "ControlChannel.h"

#include "Intake.h"
#include "FaultManager.h"
#include "SwerveController.h"

enum class LiftMechanismState {
    park,
    pick,
    place, 
    flippped_park,
    algae,
    mid
};

enum class LiftDetailedState {
    tower,
    prep_tower,
    prep_pick,
    pick,
    prep_place,
    prep_algae,
    algae,
    place,
    eject_coral,
    park,
    flipped_park,
    prep_flipped_park,
    mid
};

enum class ShoulderCalibrationState {
    uncalibrated,
    calibrating,
    calibrated,
    failed
};

class Lift : public Task
{
private:

    ctre::phoenix6::hardware::TalonFX lift_motor = ctre::phoenix6::hardware::TalonFX(10, "rio");
    ctre::phoenix6::hardware::TalonFX lift_follower = ctre::phoenix6::hardware::TalonFX(11, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration lift_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(14.54)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(11.5_V) //TODO: increase limits
            .WithPeakReverseVoltage(-11.5_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(25_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(5.0_tps)
            .WithMotionMagicAcceleration(18_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(40).WithKI(0).WithKD(0.1)
            .WithKS(0.2).WithKV(1.86).WithKG(0.25).WithKA(0.05)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
        );

    const units::angle::turn_t max_lift_position = 5.0_tr;
    const units::angle::turn_t lift_park_position = 0.1_tr;

    const units::angle::turn_t lift_min_bay_position = 1.3_tr; //minimum position if the claw is in the bay
    const units::angle::turn_t lift_min_bay_traverse_position = 1.35_tr; //minimum position if the claw is 

    const units::angle::turn_t lift_intake_liftonly_clearance = 1.15_tr;
    const units::angle::turn_t lift_intake_bay_clearance = 0.45_tr;

    const units::angle::turn_t lift_tower_position = 3.2_tr;
    const units::angle::turn_t lift_tower_bayopen_position = 2.9_tr;
    const units::angle::turn_t lift_pick_position = 1.38_tr;
    const units::angle::turn_t lift_flipped_park_position = 3.0_tr;

    const units::angle::turn_t lift_place_position = 2.05_tr;
    const units::angle::turn_t lift_algae_position = 3.15_tr;

    const units::angle::turn_t lift_place_positions[4] = {
        1.98_tr,
        1.98_tr,
        1.98_tr,
        3.75_tr,
    };

    const units::angle::turn_t lift_algae_positions[4] = {
        1.90_tr,
        1.90_tr,
        3.12_tr,
        3.12_tr,
    };

    ctre::phoenix6::StatusSignal<units::angle::turn_t> lift_position = lift_motor.GetPosition();

    ctre::phoenix6::controls::MotionMagicVoltage lift_control = ctre::phoenix6::controls::MotionMagicVoltage(0_tr);

    ctre::phoenix6::hardware::TalonFX shoulder_motor = ctre::phoenix6::hardware::TalonFX(20, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration shoulder_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(81.45)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(8.5_V) //TODO: increase limits
            .WithPeakReverseVoltage(-8.5_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(30_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(3.0_tps)
            .WithMotionMagicAcceleration(3.6_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(65).WithKI(0).WithKD(0.1)
            .WithKS(0.2).WithKV(1.6).WithKG(0.25).WithKA(0.06)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
        );

    const units::angle::turn_t shoulder_park_position = 0_tr;
    const units::angle::turn_t shoulder_max_position = 0.52_tr;

    const units::angle::turn_t shoulder_clear_position = 0.050_tr;
    const units::angle::turn_t shoulder_bay_exit_limit = 0.49_tr; //when the claw is in the bay
    const units::angle::turn_t shoulder_bay_enter_limit = 0.30_tr; //when the claw is NOT in the bay

    const units::angle::turn_t shoulder_intake_liftonly_clearance = 0.06_tr;

    const units::angle::turn_t shoulder_level_position = 0.25_tr;

    const units::angle::turn_t shoulder_pick_position = 0.51_tr;
    const units::angle::turn_t shoulder_place_position = 0.17_tr;
    const units::angle::turn_t shoulder_algae_position = 0.175_tr;

    ctre::phoenix6::StatusSignal<units::angle::turn_t> shoulder_motor_position = shoulder_motor.GetPosition();

    ctre::phoenix6::controls::MotionMagicVoltage shoulder_control = ctre::phoenix6::controls::MotionMagicVoltage(shoulder_park_position);

    ctre::phoenix6::hardware::CANcoder shoulder_encoder = ctre::phoenix6::hardware::CANcoder(20, "rio");

    ctre::phoenix6::configs::CANcoderConfiguration shoulder_encoder_configuration = ctre::phoenix6::configs::CANcoderConfiguration()
        .WithMagnetSensor(ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithMagnetOffset(0.1413_tr)
        );

    ctre::phoenix6::StatusSignal<units::angle::turn_t> shoulder_encoder_position = shoulder_motor.GetPosition();

    ctre::phoenix6::hardware::TalonFX twist_motor = ctre::phoenix6::hardware::TalonFX(21, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration twist_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithSensorToMechanismRatio(10)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(10.2_V) //TODO: increase limits
            .WithPeakReverseVoltage(-10.2_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(12_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
            .WithMotionMagicCruiseVelocity(3.0_tps)
            .WithMotionMagicAcceleration(10.0_tr_per_s_sq)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(80).WithKI(0).WithKD(0.1)
            .WithKS(0.1).WithKV(1.0).WithKG(0.0).WithKA(0.1)
        );

    ctre::phoenix6::hardware::TalonFX gripper_motor = ctre::phoenix6::hardware::TalonFX(22, "rio");

    ctre::phoenix6::configs::TalonFXConfiguration gripper_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(10_V) //TODO: increase limits
            .WithPeakReverseVoltage(-10_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(25.5_A)
            .WithStatorCurrentLimitEnable(true)
        );

    ctre::phoenix6::controls::VoltageOut gripper_control = ctre::phoenix6::controls::VoltageOut(0.0_V);

    grpl::LaserCan gripper_sensor = grpl::LaserCan(20);

    const uint16_t gripper_coral_threshold = 35;

    //state

    bool gripper_coral;

    ShoulderCalibrationState rotate_calibration_state = ShoulderCalibrationState::uncalibrated; //and synced
    std::chrono::time_point<std::chrono::steady_clock> rotate_calibration_start;

    std::chrono::time_point<std::chrono::steady_clock> eject_start;

    units::angle::turn_t target_shoulder_position = shoulder_park_position;
    units::angle::turn_t target_lift_position = lift_park_position;
    units::angle::turn_t target_twist_position = 0_tr;

    LiftMechanismState current_mechanism_state = LiftMechanismState::park;
    LiftDetailedState detailed_state = LiftDetailedState::park;

    bool lift_locked;
    units::angle::turn_t lift_lock_position = lift_park_position;

    controlchannel::ControlHandle<intake::IntakeClearanceLevel> get_out_handle;
    intake::Intake *intake;

    controlchannel::ControlHandle<LateralSwerveRequest> lateral_drive_handle;

    FaultManager fault_manager = FaultManager("lift");

    std::shared_ptr<nt::NetworkTable> ui_table = nt::NetworkTableInstance(nt::GetDefaultInstance()).GetTable("lift");

    units::angle::turn_t filter_lift_position(units::angle::turn_t input);
    units::angle::turn_t filter_shoulder_position(units::angle::turn_t input);

    void handle_pick();
    void handle_place();
    void handle_park();
    void handle_mid();
    void handle_flipped_park();
    void handle_algae();

public:

    Lift(intake::Intake *intake, controlchannel::ControlHandle<LateralSwerveRequest> lateral_drive_handle, GlobalFaultManager *global_fm);

    controlchannel::ControlChannel<LiftMechanismState> target_mechanism_state = controlchannel::ControlChannel<LiftMechanismState>(LiftMechanismState::park);

    controlchannel::ControlChannel<int> target_place_position = controlchannel::ControlChannel<int>(3);

    LiftDetailedState get_state();

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    ~Lift();
};
