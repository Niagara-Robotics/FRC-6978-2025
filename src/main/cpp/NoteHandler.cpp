#include "NoteHandler.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cstdio>
#include <iostream>

NoteHandler::NoteHandler() {
    //launcher
    ctre::phoenix6::configs::TalonFXConfiguration launcher_config = ctre::phoenix6::configs::TalonFXConfiguration();
    launcher_config.CurrentLimits.StatorCurrentLimit = 15;
    launcher_config.CurrentLimits.StatorCurrentLimitEnable = true;
    launcher_config.Slot0.kP = 0.3;
    launcher_config.Slot0.kV = 7.04 / 62;
    launcher_config.Slot0.kS = 0.15;

    launcher_request.UpdateFreqHz = 0_Hz;
    
    lower_left_launcher_motor.GetConfigurator().Apply(launcher_config);
    upper_left_launcher_motor.GetConfigurator().Apply(launcher_config);
    lower_right_launcher_motor.GetConfigurator().Apply(launcher_config);
    upper_right_launcher_motor.GetConfigurator().Apply(launcher_config);

    lower_left_launcher_motor.SetInverted(false);
    upper_left_launcher_motor.SetInverted(false);
    lower_right_launcher_motor.SetInverted(true);
    upper_right_launcher_motor.SetInverted(true);

    upper_left_launcher_velocity = &upper_left_launcher_motor.GetVelocity();
    upper_right_launcher_velocity = &upper_right_launcher_motor.GetVelocity();
    lower_left_launcher_velocity = &lower_left_launcher_motor.GetVelocity();
    lower_right_launcher_velocity = &lower_right_launcher_motor.GetVelocity();

    upper_left_launcher_velocity->SetUpdateFrequency(180_Hz);
    upper_right_launcher_velocity->SetUpdateFrequency(180_Hz);
    lower_left_launcher_velocity->SetUpdateFrequency(180_Hz);
    lower_right_launcher_velocity->SetUpdateFrequency(180_Hz);

    set_launcher_motors();

    //trajectory control
    
    tilt_configuration.Voltage.PeakForwardVoltage = 2.5;
    tilt_configuration.Voltage.PeakReverseVoltage = -0.6;
    tilt_configuration.Slot0.kP = 95;
    tilt_configuration.Slot0.kD = 0.5;
    tilt_configuration.Slot0.kG = 0.313;
    tilt_configuration.Slot0.kA = 0.60;
    tilt_configuration.Slot0.kV = 2.7;
    tilt_configuration.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    tilt_configuration.MotionMagic.MotionMagicAcceleration = 1.3;
    tilt_configuration.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    tilt_configuration.Feedback.SensorToMechanismRatio = 50.0;
    tilt_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = tilt_max_position.value() / (2 * M_PI);
    tilt_configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = tilt_park_position.value() / (2 * M_PI);
    tilt_configuration.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    tilt_motor.GetConfigurator().Apply(tilt_configuration);
    
    tilt_position_signal = &tilt_motor.GetPosition();
    tilt_velocity_signal = &tilt_motor.GetVelocity();

    tilt_motor.GetMotorVoltage().SetUpdateFrequency(100_Hz);
    tilt_motor.GetAcceleration().SetUpdateFrequency(100_Hz);

    tilt_position_signal->SetUpdateFrequency(180_Hz);
    tilt_velocity_signal->SetUpdateFrequency(180_Hz);

    tilt_motor.OptimizeBusUtilization();

    tilt_motor.SetPosition(tilt_park_position);

    tilt_request.UpdateFreqHz = 0_Hz;
    tilt_request.Position = tilt_park_position;

    

    tilt_motor.SetControl(tilt_request);
    tilt_calibration_state = TiltCalibrationState::uncalibrated;

    //intake
    ctre::phoenix6::configs::TalonFXConfiguration intake_configuration = ctre::phoenix6::configs::TalonFXConfiguration();
    intake_configuration.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    intake_configuration.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    intake_motor.GetConfigurator().Apply(intake_configuration);

    //indexer
    ctre::phoenix6::configs::TalonFXConfiguration indexer_configuration = ctre::phoenix6::configs::TalonFXConfiguration();
    indexer_configuration.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    indexer_configuration.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    index_motor.GetConfigurator().Apply(indexer_configuration);
}

void NoteHandler::enable_tilt_softlimit() {
    tilt_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    tilt_configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    tilt_motor.GetConfigurator().Apply(tilt_configuration);
}

void NoteHandler::set_launcher_motors() {
    upper_left_launcher_motor.SetControl(launcher_request);
    upper_right_launcher_motor.SetControl(launcher_request);
    lower_left_launcher_motor.SetControl(launcher_request);
    lower_right_launcher_motor.SetControl(launcher_request);
}

void NoteHandler::idle_launcher_motors() {
    upper_left_launcher_motor.SetControl(ctre::phoenix6::controls::CoastOut());
    upper_right_launcher_motor.SetControl(ctre::phoenix6::controls::CoastOut());
    lower_left_launcher_motor.SetControl(ctre::phoenix6::controls::CoastOut());
    lower_right_launcher_motor.SetControl(ctre::phoenix6::controls::CoastOut());
}

void NoteHandler::update_tilt_interlock() {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(*tilt_position_signal, *tilt_velocity_signal);
    tilt_interlock = fabs((tilt_channel.get() - tilt_position_signal->GetValue()).value()) < tilt_position_tolerance.value() &&
        fabs(tilt_velocity_signal->GetValue().value()) < tilt_velocity_tolerance.value();
    //TODO: add other safety stuff
    frc::SmartDashboard::PutBoolean("tilt_interlock", tilt_interlock);
}

void NoteHandler::update_launcher_velocity_interlock() {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(
        *upper_left_launcher_velocity, 
        *upper_right_launcher_velocity, 
        *lower_left_launcher_velocity, 
        *lower_right_launcher_velocity);
    
    launcher_velocity_interlock = 
        (fabs(launcher_velocity_channel.get().value() - upper_left_launcher_velocity->GetValue().value()) < launcher_velocity_tolerance.value()) &&
        (fabs(launcher_velocity_channel.get().value() - upper_right_launcher_velocity->GetValue().value()) < launcher_velocity_tolerance.value()) &&
        (fabs(launcher_velocity_channel.get().value() - lower_left_launcher_velocity->GetValue().value()) < launcher_velocity_tolerance.value()) &&
        (fabs(launcher_velocity_channel.get().value() - lower_right_launcher_velocity->GetValue().value()) < launcher_velocity_tolerance.value());
}

units::volt_t NoteHandler::calculate_appropriate_indexer_voltage() {
    return 0.35_V+(0.12_V*launcher_velocity_channel.get().value());
}

bool NoteHandler::get_fire_timer_elapsed(){
    return fire_timer_elapsed;
}

IntakeIndexingState NoteHandler::get_index_state() {
    return indexing_state;
}

void NoteHandler::call(bool robot_enabled, bool autonomous) {
    switch (tilt_calibration_state)
    {
    case TiltCalibrationState::uncalibrated:
        if(robot_enabled) { //cannot calibrate while the bot is disabled as it requires output
            tilt_motor.SetControl(ctre::phoenix6::controls::VoltageOut(0.3_V));
            tilt_calibration_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
            tilt_calibration_state = TiltCalibrationState::calibrating;
            std::cout << "Started calibration" << std::endl;
        }
        break;
    //non-blocking calibration process, will still accept commands but wont be relayed to motor
    case TiltCalibrationState::calibrating:
        if((std::chrono::steady_clock::now() > tilt_calibration_end)) {
            std::cout << "Ended calibration" << std::endl;
            tilt_motor.SetPosition(tilt_park_position);
            tilt_request.Position = tilt_park_position;
            tilt_motor.SetControl(tilt_request);
            enable_tilt_softlimit();
            tilt_calibration_state = TiltCalibrationState::calibrated;
        }
        break;
    
    case TiltCalibrationState::calibrated:
        update_tilt_interlock();
        tilt_request.Position = tilt_channel.get();
        if(tilt_request.Position > tilt_max_position) tilt_request.Position = tilt_max_position;
        if(tilt_request.Position < tilt_park_position) tilt_request.Position = tilt_park_position;
        tilt_motor.SetControl(tilt_request);
        frc::SmartDashboard::PutNumber("tilt_position", tilt_position_signal->GetValueAsDouble() * 2 * M_PI);
        break;
    }
    frc::SmartDashboard::PutNumber("tilt_calibration_state", (int)tilt_calibration_state);

    switch (launcher_state)
    {
    case LauncherState::idle:
        idle_launcher_motors();
        launcher_velocity_interlock = false; //interlock not possible with the motors in idle
        
        if(launcher_mode_channel.get() != LauncherMode::idle && indexing_state == IntakeIndexingState::hold) launcher_state = LauncherState::waiting;
        if(launcher_mode_channel.get() == LauncherMode::emergency) launcher_state = LauncherState::firing;
        fire_timer_elapsed = false;
        break;

    case LauncherState::waiting:
        launcher_request.Velocity = launcher_velocity_channel.get();
        set_launcher_motors();
        update_launcher_velocity_interlock();
        switch (launcher_mode_channel.get())
        {
        case LauncherMode::velocity_interlock:
            if(launcher_velocity_interlock) launcher_state = LauncherState::firing;
            fire_start = std::chrono::steady_clock::now();
            break;
        
        case LauncherMode::tilt_interlock:
            if(launcher_velocity_interlock && tilt_interlock) launcher_state = LauncherState::firing;
            fire_start = std::chrono::steady_clock::now();
            break;
        }

        if(launcher_mode_channel.get() == LauncherMode::idle) {
            idle_launcher_motors();
            indexing_state = IntakeIndexingState::empty;
            launcher_state = LauncherState::idle;
        }
        fire_timer_elapsed = false;
        break;
    case LauncherState::firing:
        indexing_state = IntakeIndexingState::rolling_out;
        set_launcher_motors();
        if(launcher_mode_channel.get() == LauncherMode::idle) {
            idle_launcher_motors();
            indexing_state = IntakeIndexingState::empty;
            launcher_state = LauncherState::idle;
        }
        fire_timer_elapsed = std::chrono::steady_clock::now() > (fire_start+std::chrono::seconds(1));
        break;
    }

    switch(indexing_state) {
        case IntakeIndexingState::rolling_in:
            intake_motor.SetControl(intake_request);
            index_motor.SetControl(ctre::phoenix6::controls::VoltageOut(1.8_V));
            if(!index_sensor.Get()) indexing_state = IntakeIndexingState::hold;
            break;
        case IntakeIndexingState::rolling_out:
            intake_motor.SetControl(ctre::phoenix6::controls::StaticBrake());
            index_motor.SetControl(ctre::phoenix6::controls::VoltageOut(launcher_velocity_channel.get().value() * 0.2_V));
            break;
        case IntakeIndexingState::empty:
            if(indexing_mode_channel.get() == IntakeIndexingMode::roll_in) {
                indexing_state = IntakeIndexingState::rolling_in;
            }
            intake_motor.SetControl(ctre::phoenix6::controls::StaticBrake());
            index_motor.SetControl(ctre::phoenix6::controls::StaticBrake());
            if(!index_sensor.Get()) indexing_state = IntakeIndexingState::hold;
            break;
        case IntakeIndexingState::hold:
            intake_motor.SetControl(ctre::phoenix6::controls::StaticBrake());
            index_motor.SetControl(ctre::phoenix6::controls::StaticBrake());
            break;
    }
    
    frc::SmartDashboard::PutNumber("indexing_state", (int)indexing_state);
    frc::SmartDashboard::PutNumber("indexing_mode", (int)indexing_mode_channel.get());
}

void NoteHandler::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(5500);
}

bool NoteHandler::is_paused() {
    return false;
}

NoteHandler::~NoteHandler() {
}