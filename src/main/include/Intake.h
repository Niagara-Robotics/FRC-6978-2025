#include "Task.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class Intake  : public Task
{
private:
    ctre::phoenix6::hardware::TalonFX rotate_motor = ctre::phoenix6::hardware::TalonFX(30, "rio");
    ctre::phoenix6::hardware::TalonFX vertical_motor = ctre::phoenix6::hardware::TalonFX(31, "rio");
    ctre::phoenix6::hardware::TalonFX horizontal_motor = ctre::phoenix6::hardware::TalonFX(32, "rio");

    const ctre::phoenix6::configs::TalonFXConfiguration rotate_config = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs()
            .WithRotorToSensorRatio(100)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
            .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        )
        .WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
            .WithPeakForwardVoltage(0.5_V)
            .WithPeakReverseVoltage(-0.5_V)
        )
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(20_A)
            .WithStatorCurrentLimitEnable(true)
        )
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs()
            .WithKP(100).WithKI(0).WithKD(0.5)
            .WithKS(0.2).WithKV(2.0).WithKG(0.3)
        )
        ;
public:
    Intake(/* args */);
    ~Intake();

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;
};
