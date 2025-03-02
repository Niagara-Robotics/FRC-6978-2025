#include "Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace intake;

void Intake::configure_rotate_motor() {
    if (rotate_motor.GetConfigurator().Apply(rotate_config) != ctre::phoenix::StatusCode::OK)
        fault_manager.add_fault(Fault(true, FaultIdentifier::intakeRotateUnreachable));
    rotate_motor.SetExpiration(40_ms);
    rotate_motor.SetSafetyEnabled(true);

    rotate_motor_position.SetUpdateFrequency(100_Hz);
    rotate_temperature.SetUpdateFrequency(10_Hz);
    rotate_current.SetUpdateFrequency(100_Hz);

    rotate_motor.SetPosition(rotate_encoder.GetAbsolutePosition().GetValue());

    //rotate_motor.OptimizeBusUtilization();

    rotate_calibration_state = RotateCalibrationState::uncalibrated;

    rotate_control.UpdateFreqHz = 0_Hz;
    printf("Configured intake rotation\n");
}

void Intake::enable_rotate_softlimit() {
    if (rotate_motor.GetConfigurator().Apply(rotate_config.WithSoftwareLimitSwitch(rotate_softlimit_config)) != ctre::phoenix::StatusCode::OK)
        fault_manager.add_fault(Fault(true, FaultIdentifier::intakeRotateUnreachable));
}

Intake::Intake() {
    configure_rotate_motor();

    if (rotate_encoder.GetConfigurator().Apply(rotate_encoder_config) != ctre::phoenix::StatusCode::OK)
        fault_manager.add_fault(Fault(true, FaultIdentifier::intakeRotateEncoderUnreachable));

    vertical_motor.GetConfigurator().Apply(horizontal_vertical_config);
    horizontal_motor.GetConfigurator().Apply(horizontal_vertical_config);

}

bool Intake::is_clear_lift() {
    if(fault_manager.get_fault(FaultIdentifier::intakeRotateUnreachable, nullptr) || 
        fault_manager.get_fault(FaultIdentifier::intakeRotateThermalLimit, nullptr))
        return false;
    ctre::phoenix6::BaseStatusSignal::RefreshAll(rotate_encoder_position);

    return rotate_encoder_position.GetValue() > rotate_lift_clearance_position;
}

void Intake::handle_pickup_algae() {
    switch (state)
    {
    case IntakeState::standby:
        rotate_target_position = algae_pickup_position;
        vertical_control.Output = vertical_algae_pickup_voltage;
        horizontal_control.Output = horizontal_algae_pickup_voltage;
        state = IntakeState::algae_pickup;
        break;
    
    case IntakeState::algae_pickup:
        vertical_control.Output = vertical_algae_pickup_voltage;
        horizontal_control.Output = horizontal_algae_pickup_voltage;
        rotate_target_position = algae_pickup_position;
        if(intake_sensor.get_measurement().has_value())
            if(intake_sensor.get_measurement().value().distance_mm < algae_lock_threshold &&
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT &&
                intake_sensor.get_measurement().value().ambient < 32)
                state = IntakeState::algae_locked;
            else break;
        else break;
    
    case IntakeState::algae_locked:
        vertical_control.Output = (abs(rotate_motor_position.GetValueAsDouble() - algae_hold_position.value()) < 0.01)?  vertical_algae_hold_voltage : vertical_algae_pickup_voltage;
        rotate_target_position = algae_hold_position;
        horizontal_control.Output = 0_V;
        if(intake_sensor.get_measurement().has_value())
            if((intake_sensor.get_measurement().value().distance_mm > algae_lock_threshold + 65 &&
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) ||
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_OUT_OF_BOUNDS) {
                state = IntakeState::standby;
                intake_action_channel.take_control(0, true);
                intake_action_channel.set(0, IntakeAction::standby);
            }
                
        break;
    
    default:
        state = IntakeState::standby;
        break;
    }
}

void Intake::handle_eject_algae() {
    switch (state)
    {
    case IntakeState::algae_locked:
        vertical_control.Output = vertical_algae_eject_voltage;
        rotate_target_position = algae_hold_position;
        horizontal_control.Output = 0_V;
        if(intake_sensor.get_measurement().has_value())
            if(intake_sensor.get_measurement().value().distance_mm > algae_lock_threshold + 65 &&
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
                state = IntakeState::standby;
                intake_action_channel.take_control(0, true);
                intake_action_channel.set(0, IntakeAction::standby);
            }
        break;
    
    default:
        intake_action_channel.take_control(0, true);
        intake_action_channel.set(0, IntakeAction::standby);
        break;
    }
}

void Intake::handle_pickup_coral() {
    switch (state)
    {
    case IntakeState::standby:
        rotate_target_position = coral_vertical_a_position;
        vertical_control.Output = vertical_coral_a_voltage;
        horizontal_control.Output = 0_V;
        state = IntakeState::coral_vertical_a;
        break;
    case IntakeState::coral_vertical_a:
        rotate_target_position = coral_vertical_a_position;
        vertical_control.Output = vertical_coral_a_voltage;
        horizontal_control.Output = 0_V;
        if(intake_sensor.get_measurement().has_value())
            if(intake_sensor.get_measurement().value().distance_mm < coral_a_threshold &&
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT &&
                intake_sensor.get_measurement().value().ambient < 32)
                state = IntakeState::coral_horizontal;
            else break;
        else break;
    case IntakeState::coral_horizontal:
        rotate_target_position = coral_horizontal_position;
        vertical_control.Output = 0_V;
        horizontal_control.Output = horizontal_coral_voltage;
        if(intake_sensor.get_measurement().has_value())
            if(intake_sensor.get_measurement().value().distance_mm < coral_horizontal_threshold &&
                intake_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT &&
                intake_sensor.get_measurement().value().ambient < 32)
                state = IntakeState::coral_vertical_b;
        break;
    case IntakeState::coral_vertical_b:
        rotate_target_position = coral_vertical_b_position;
        vertical_control.Output = vertical_coral_b_voltage;
        horizontal_control.Output = 0_V;
        if(staging_sensor.get_measurement().has_value())
            if(staging_sensor.get_measurement().value().distance_mm < coral_hold_threshold &&
                staging_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT &&
                staging_sensor.get_measurement().value().ambient < 32){
                state = IntakeState::standby;
                intake_action_channel.take_control(0, true);
                intake_action_channel.set(0, IntakeAction::standby);
            }
        break;
    default:
        break;
    }
}

void Intake::call(bool robot_enabled, bool autonomous) {
    rotate_motor.Feed();
    fault_manager.feed_watchdog();

    if(intake_sensor.get_measurement().has_value()) {
        frc::SmartDashboard::PutNumber("intake_sensor", intake_sensor.get_measurement().value().distance_mm);
        frc::SmartDashboard::PutNumber("intake_sensor_ambient", intake_sensor.get_measurement().value().ambient);
    }
    
    switch (intake_action_channel.get())
    {
    case IntakeAction::pickup_algae:
        handle_pickup_algae(); //done ish
        break;
    case IntakeAction::pickup_coral:
        handle_pickup_coral();
        break;
    case IntakeAction::eject_algae:
        handle_eject_algae();
        break;
    default:
        rotate_target_position = rotate_physical_stop_position+ 5_deg;
        vertical_control.Output = 0_V;
        horizontal_control.Output = 0_V;
        state=IntakeState::standby;
        break;
    }

    

    //safety checks
    
    if(!rotate_motor.IsConnected(450_ms) || !rotate_motor.IsSafetyEnabled()) {
        rotate_motor.Disable();
        rotate_calibration_state = RotateCalibrationState::uncalibrated;
        fault_manager.add_fault(Fault(true, FaultIdentifier::intakeRotateUnreachable));
        return;
    }

    if(fault_manager.get_fault(FaultIdentifier::intakeRotateUnreachable, nullptr)) {
        if(rotate_motor.IsConnected(450_ms)) {
            fault_manager.clear_fault(Fault(false, FaultIdentifier::intakeRotateUnreachable));
            configure_rotate_motor();
        }
        return;
    }

    ctre::phoenix6::BaseStatusSignal::RefreshAll(rotate_motor_position, rotate_encoder_position, rotate_current, rotate_temperature);

    if(rotate_temperature.GetValue() > 55_degC) {
        fault_manager.add_fault(Fault(true, FaultIdentifier::intakeRotateThermalLimit));
        rotate_motor.Disable();
        return;
    } else {
        fault_manager.clear_fault(Fault(false, FaultIdentifier::intakeRotateThermalLimit));
    }

    if(!robot_enabled) return;

    switch (rotate_calibration_state)
    {
    case RotateCalibrationState::uncalibrated:
        rotate_motor.SetControl(ctre::phoenix6::controls::VoltageOut(-0.15_V));
        rotate_calibration_start = std::chrono::steady_clock::now();
        horizontal_motor.Disable();
        vertical_motor.Disable();
        rotate_calibration_state = RotateCalibrationState::calibrating;
        break;
    
    case RotateCalibrationState::calibrating:
        rotate_motor.SetControl(ctre::phoenix6::controls::VoltageOut(-0.15_V));
        //fail if the calibration doesnt satisfy after 3 seconds
        if(std::chrono::steady_clock::now() - rotate_calibration_start > std::chrono::seconds(3)) {
            fault_manager.add_fault(Fault(false, FaultIdentifier::intakeRotateCalibrationTimeout));
            rotate_calibration_state = RotateCalibrationState::failed;
            rotate_motor.Disable();
        }
        //must be on-time, on position, and stopped
        if(std::chrono::steady_clock::now() - rotate_calibration_start > std::chrono::milliseconds(800) &&
            abs(rotate_encoder.GetVelocity().GetValueAsDouble()) < 0.08 &&
            rotate_encoder_position.GetValue() < rotate_physical_stop_position + 8_deg) 
        {
            rotate_motor.SetPosition(rotate_encoder_position.GetValue());
            enable_rotate_softlimit();
            rotate_calibration_state = RotateCalibrationState::finished;
        }
        break;
    
    case RotateCalibrationState::finished:
        //filter the target position
        rotate_target_position = (rotate_target_position > rotate_maximum_extent)? rotate_maximum_extent : rotate_target_position;
        rotate_target_position = (rotate_target_position < rotate_physical_stop_position)? rotate_physical_stop_position : rotate_target_position;
        
        //check if the lift wants the intake out of the way, and limit the target position as such
        if(lift_clearance_channel.get()) {
            rotate_target_position = (rotate_target_position < rotate_lift_clearance_position)? rotate_lift_clearance_position : rotate_target_position;
        }

        rotate_control.Position = rotate_target_position;

        rotate_motor.SetControl(rotate_control);
        vertical_motor.SetControl(vertical_control);
        horizontal_motor.SetControl(horizontal_control);
        break;

    default:
        break;
    }
    
    
    return;
}

void Intake::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::milliseconds(10);
}

bool Intake::is_paused() {
    return false;
}

Intake::~Intake() {

}