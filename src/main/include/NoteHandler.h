#pragma once

#include "Task.h"
#include "ControlChannel.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include "ControlChannel.h"
#include <frc/DigitalInput.h>

#include "FaultManager.h"

using namespace ctre::phoenix6::hardware;

enum class TiltCalibrationState {
    uncalibrated,
    calibrating,
    calibrated
};

enum class LauncherMode {
    tilt_interlock, //fire when the tilt mechanism reports stable and the previous velocity interlock is met
    velocity_interlock, //fire when the velocity meets the target
    emergency, //no checks, just fire the darn thing
    spin, //just spin the motors at the requested velocity
    idle,
};

enum class LauncherState {
    firing,
    waiting,
    idle,
};

enum class IntakeIndexingState {
    empty,
    rolling_in,
    hold,
    rolling_out
};

enum class IntakeIndexingMode {
    stop,
    roll_in,
    roll_out
};

class NoteHandler: public Task
{
private:
    //config
    const units::angle::radian_t tilt_max_position = 0.75_rad;
    const units::angle::radian_t tilt_park_position = 0.113446_rad;

    const units::angle::radian_t tilt_position_tolerance = 0.03_rad;
    const units::angular_velocity::radians_per_second_t tilt_velocity_tolerance = 0.008_rad_per_s;

    const units::angular_velocity::turns_per_second_t launcher_velocity_tolerance = 0.8_tps;

    //hardware
    TalonFX upper_left_launcher_motor = TalonFX(20, "rio");
    TalonFX upper_right_launcher_motor = TalonFX(22, "rio");

    TalonFX lower_left_launcher_motor = TalonFX(21, "rio");
    TalonFX lower_right_launcher_motor = TalonFX(23, "rio");

    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *upper_left_launcher_velocity;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *upper_right_launcher_velocity;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *lower_left_launcher_velocity;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *lower_right_launcher_velocity;

    TalonFX tilt_motor = TalonFX(30, "rio");
    ctre::phoenix6::configs::TalonFXConfiguration tilt_configuration = ctre::phoenix6::configs::TalonFXConfiguration();
    
    ctre::phoenix6::StatusSignal<units::angle::turn_t> *tilt_position_signal;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *tilt_velocity_signal;

    TalonFX index_motor = TalonFX(40, "rio");

    TalonFX intake_motor = TalonFX(10, "rio");

    ctre::phoenix6::controls::VelocityVoltage launcher_request = ctre::phoenix6::controls::VelocityVoltage(0.0_tps);
    ctre::phoenix6::controls::MotionMagicVoltage tilt_request = ctre::phoenix6::controls::MotionMagicVoltage(0.0_rad);

    ctre::phoenix6::controls::VoltageOut intake_request = ctre::phoenix6::controls::VoltageOut(4.2_V);

    //sensor
    frc::DigitalInput index_sensor = frc::DigitalInput(0);

    //state
    TiltCalibrationState tilt_calibration_state = TiltCalibrationState::uncalibrated;
    std::chrono::time_point<std::chrono::steady_clock> tilt_calibration_end;
    bool tilt_interlock;

    LauncherState launcher_state = LauncherState::idle;
    bool launcher_velocity_interlock;

    IntakeIndexingState indexing_state = IntakeIndexingState::empty;
    IntakeIndexingMode indexing_mode = IntakeIndexingMode::stop;

    std::chrono::time_point<std::chrono::steady_clock> fire_start;
    bool fire_timer_elapsed = false;

    //helpers
    void set_launcher_motors();
    void idle_launcher_motors();
    void update_tilt_interlock();
    void update_launcher_velocity_interlock();
    void enable_tilt_softlimit();
    units::volt_t calculate_appropriate_indexer_voltage();

    FaultManager fault_manager = FaultManager("note_handler");
public:
    NoteHandler(GlobalFaultManager *global_fault_manager);

    controlchannel::ControlChannel<units::angle::radian_t> tilt_channel = controlchannel::ControlChannel(0.0_rad);
    controlchannel::ControlChannel<LauncherMode> launcher_mode_channel = controlchannel::ControlChannel(LauncherMode::idle);
    controlchannel::ControlChannel<units::angular_velocity::turns_per_second_t> launcher_velocity_channel = controlchannel::ControlChannel(75.0_tps);
    controlchannel::ControlChannel<IntakeIndexingMode> indexing_mode_channel = controlchannel::ControlChannel(IntakeIndexingMode::stop);

    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    IntakeIndexingState get_index_state();

    bool get_tilt_interlock();
    LauncherState get_launcher_state();
    bool get_fire_timer_elapsed();

    ~NoteHandler();
};
