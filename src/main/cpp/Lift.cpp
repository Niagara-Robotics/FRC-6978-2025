#include "Lift.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "string.h"
#include <hal/HALBase.h>

Lift::Lift(intake::Intake *intake,controlchannel::ControlHandle<LateralSwerveRequest> lateral_drive_handle, GlobalFaultManager *global_fm): intake(intake),get_out_handle(intake->get_outta_the_way_channel.get_handle()),lateral_drive_handle(lateral_drive_handle) {
    global_fm->register_manager(&fault_manager);
    
    lift_motor.GetConfigurator().Apply(lift_config);
    shoulder_motor.GetConfigurator().Apply(shoulder_config);
    shoulder_encoder.GetConfigurator().Apply(shoulder_encoder_configuration);

    shoulder_motor_position.SetUpdateFrequency(100_Hz);
    shoulder_encoder_position.SetUpdateFrequency(100_Hz);
    lift_position.SetUpdateFrequency(100_Hz);

    shoulder_motor.SetPosition(shoulder_encoder.GetAbsolutePosition().GetValue());

    lift_motor.SetPosition(0_tr);

    lift_control.UpdateFreqHz = 0_Hz;

    twist_motor.GetConfigurator().Apply(twist_config);
    twist_motor.SetPosition(0_tr);

    gripper_motor.GetConfigurator().Apply(gripper_config);
}

units::angle::turn_t Lift::filter_lift_position(units::angle::turn_t input) {
    if(lift_locked) {
        return lift_lock_position;
    }

    ctre::phoenix6::BaseStatusSignal::RefreshAll(shoulder_motor_position, lift_position, shoulder_encoder_position);
    input = (input > max_lift_position)? max_lift_position: input;
    input = (input < lift_park_position)? lift_park_position: input;
    //shoulder is in the bay traverse zone
    if(shoulder_encoder_position.GetValue() > shoulder_bay_enter_limit && shoulder_encoder_position.GetValue() < shoulder_bay_exit_limit)
        input = (input < lift_min_bay_traverse_position)? lift_min_bay_traverse_position : input;
    //shoulder is in the bay
    else if(shoulder_encoder_position.GetValue() > shoulder_bay_exit_limit) 
        input = (input < lift_min_bay_position)? lift_min_bay_position : input;
    else //shoulder is NOT in the bay
        //lock to the park position if the shoulder is in or the intake is not clear
        input = (shoulder_encoder_position.GetValue() < shoulder_clear_position ||
            ( intake->get_clearance_level() == intake::IntakeClearanceLevel::none && lift_position.GetValue() < lift_intake_liftonly_clearance))? lift_park_position : input;
    
    if(intake->get_clearance_level() == intake::IntakeClearanceLevel::lift_only)
        if(shoulder_encoder_position.GetValue() > shoulder_bay_exit_limit)
            input = (input < lift_flipped_park_position)? lift_flipped_park_position : input;
        else if(shoulder_encoder_position.GetValue() > shoulder_intake_liftonly_clearance)
            input = (input < lift_intake_liftonly_clearance)? lift_intake_liftonly_clearance : input;

    if(intake->get_clearance_level() == intake::IntakeClearanceLevel::bay)
        if(shoulder_encoder_position.GetValue() > shoulder_bay_exit_limit)
            input = (input < lift_min_bay_position)? lift_min_bay_position : input;
        else if(shoulder_encoder_position.GetValue() > shoulder_level_position)
            input = (input < lift_min_bay_traverse_position)? lift_min_bay_traverse_position : input;
    
    //if the lift is NOT parked and the intake is NOT clear, there is something very wrong! lock the lift position!
    /*if(lift_position.GetValue() > lift_park_position + 5_deg && intake->get_clearance_level() == intake::IntakeClearanceLevel::none) {
        input = lift_position.GetValue();
        lift_lock_position = lift_position.GetValue();
        lift_locked = true;
        fault_manager.add_fault(Fault(true,FaultIdentifier::liftIntakeCollisionLock));
        printf("Lift LOCKED!\n");
    }*/

    return input;
}

units::angle::turn_t Lift::filter_shoulder_position(units::angle::turn_t input) {
    input = (input > shoulder_max_position)? shoulder_max_position: input;
    input = (input < shoulder_park_position)? shoulder_park_position: input;

    

    //lift is below the intake, intake is in lift only clearance
    if(intake->get_clearance_level() == intake::IntakeClearanceLevel::lift_only)
        if(lift_position.GetValue() < lift_intake_liftonly_clearance)
            input = (input > shoulder_intake_liftonly_clearance)? shoulder_intake_liftonly_clearance - 2.5_deg : input;
        else if(lift_position.GetValue() >= (lift_flipped_park_position - 15_deg))
            input = (input > shoulder_max_position)? shoulder_max_position: input;
        //else //above intake, can go to level
            
            //input = (input > shoulder_level_position)? shoulder_level_position - 2.5_deg : input;


    //lift is below the intake, intake is in bay clearance
    if(intake->get_clearance_level() == intake::IntakeClearanceLevel::bay) 
        if(lift_position.GetValue() <= lift_intake_bay_clearance) //below intake
            input = (input > shoulder_intake_liftonly_clearance)? shoulder_intake_liftonly_clearance : input;
        else if(lift_position.GetValue() >= lift_min_bay_traverse_position)
            input = (input > shoulder_max_position)? shoulder_max_position: input; //this is the ONLY case the max limit is allowed
        else if(shoulder_encoder_position.GetValue() > shoulder_bay_enter_limit && lift_position.GetValue() <= lift_min_bay_traverse_position) //in the bay
            input = (input < shoulder_bay_exit_limit)? shoulder_bay_exit_limit + 2.5_deg: input;
        else 
            input = (input > shoulder_level_position)? shoulder_level_position - 2.5_deg: input;

    if(intake->get_clearance_level() == intake::IntakeClearanceLevel::none)
        if(lift_position.GetValue() <= lift_intake_liftonly_clearance)
            input = shoulder_park_position;
        else 
            input = (input > shoulder_level_position)? shoulder_level_position: input;

    if(lift_position.GetValue() > lift_park_position + 5_deg)
        input = (input < shoulder_clear_position)? shoulder_clear_position : input;

    return input;

}

void Lift::handle_park() {
    switch (detailed_state)
    {
    case LiftDetailedState::park:
        target_lift_position = lift_park_position;
        target_shoulder_position = shoulder_park_position;
        break;
    case LiftDetailedState::tower:
        target_lift_position = lift_tower_position;
        target_shoulder_position = shoulder_clear_position;
        if(abs(lift_position.GetValueAsDouble() - lift_tower_position.value()) < 0.01)
            detailed_state = LiftDetailedState::park;
        break;
    case LiftDetailedState::prep_pick:
        detailed_state = LiftDetailedState::tower;
        break;
    default:
        break;
    }
}

void Lift::handle_pick() {
    switch (detailed_state)
    {
    case LiftDetailedState::park:
    case LiftDetailedState::mid:
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = shoulder_clear_position+ 15_deg;
        gripper_control.Output = 0_V;
        if(fabs(lift_position.GetValueAsDouble() - lift_tower_bayopen_position.value()) < 0.01)
            detailed_state = LiftDetailedState::tower;
        break;

    case LiftDetailedState::tower:
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = 0.5_tr;
        gripper_control.Output = 0_V;
        target_twist_position = 0.25_tr;
        detailed_state = LiftDetailedState::prep_pick;
        
        break;
    case LiftDetailedState::prep_pick: //wait
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = 0.5_tr;
        gripper_control.Output = 0_V;
        target_twist_position = 0.25_tr;
        if(intake->has_coral() && fabs(shoulder_encoder_position.GetValueAsDouble() - 0.5) < 0.005)
            detailed_state = LiftDetailedState::pick;
        break;
    case LiftDetailedState::pick:
        target_lift_position = lift_pick_position;
        gripper_control.Output = 5.0_V;
        target_shoulder_position = 0.5_tr;
        target_twist_position = 0.25_tr;
        if(gripper_coral) {
            target_mechanism_state.take_control(0, false);
            target_mechanism_state.set(0, LiftMechanismState::mid);
            printf("Backing out of pick\n");
        }
        break;
    default:
        break;
    }
}

// not really helpful
void Lift::handle_flipped_park() {
    switch (detailed_state)
    {
    case LiftDetailedState::park:
        target_lift_position = lift_tower_position;
        target_shoulder_position = shoulder_clear_position + 5_deg;
        target_twist_position = 0_tr;
        frc::SmartDashboard::PutNumber("lift_error", fabs(lift_position.GetValueAsDouble() - lift_tower_position.value()));
        detailed_state = LiftDetailedState::prep_tower;
        break;

    case LiftDetailedState::prep_tower:
        target_lift_position = lift_tower_position;
        target_shoulder_position = shoulder_clear_position + 5_deg;
        target_twist_position = 0_tr;
        frc::SmartDashboard::PutNumber("lift_error", fabs(lift_position.GetValueAsDouble() - lift_tower_position.value()));
        if(fabs(lift_position.GetValueAsDouble() - lift_tower_position.value()) < 0.01){
            detailed_state = LiftDetailedState::prep_flipped_park;
            printf("flipping\n");
        }
        
        break;

    case LiftDetailedState::prep_flipped_park:
        target_lift_position = lift_tower_position;
        target_shoulder_position = 0.5_tr;
        target_twist_position = 0.5_tr;

        //if(fabs(0.5 - shoulder_encoder_position.GetValueAsDouble()) < 0.01)
        //    detailed_state = LiftDetailedState::flipped_park;
        break;
    case LiftDetailedState::flipped_park:
        target_lift_position = lift_flipped_park_position;
        target_shoulder_position = 0.5_tr;
        break;
    default:
        break;
    }
}

void Lift::handle_mid() {
    switch (detailed_state)
    {
    case LiftDetailedState::park:
        target_lift_position = lift_intake_liftonly_clearance + 5_deg;
        target_shoulder_position = shoulder_intake_liftonly_clearance - 2_deg;
        target_twist_position = 0_tr;
        if(fabs(lift_position.GetValueAsDouble() - target_lift_position.value()) < 0.01){
            detailed_state = LiftDetailedState::mid;
        }
        break;
    case LiftDetailedState::mid:
        target_lift_position = lift_intake_liftonly_clearance + 5_deg;
        target_shoulder_position = shoulder_intake_liftonly_clearance + 0_deg;
        target_twist_position = 0.5_tr;
        gripper_control.Output = (gripper_coral)? 1_V : 0_V;
        if(target_mechanism_state.has_control(0))
            target_mechanism_state.take_control(-1, true); //release
        if(target_place_position.has_control(0))
            target_mechanism_state.take_control(-1, true);
        break;
    case LiftDetailedState::tower:
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = shoulder_clear_position + 15_deg;
        gripper_control.Output = (gripper_coral)? 1_V : 0_V;
        target_twist_position = 0.25_tr;
        if(fabs(shoulder_encoder_position.GetValueAsDouble() - (shoulder_clear_position + 15_deg).value()) < 0.01)
            detailed_state = LiftDetailedState::mid;
        break;
    case LiftDetailedState::prep_pick:
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = shoulder_intake_liftonly_clearance + 5_deg;
        gripper_control.Output = (gripper_coral)? 1_V : 0_V;
        target_twist_position = 0.5_tr;
        if(fabs(shoulder_encoder_position.GetValueAsDouble() - (shoulder_intake_liftonly_clearance + 5_deg).value()) < 0.01)
            detailed_state = LiftDetailedState::mid;
        break;
    case LiftDetailedState::pick:
        target_lift_position = lift_tower_bayopen_position;
        target_shoulder_position = 0.5_tr;
        gripper_control.Output = (gripper_coral)? 1_V : 0_V;
        target_twist_position = 0.25_tr;
        if(fabs(lift_position.GetValueAsDouble() - lift_tower_bayopen_position.value()) < 0.01)
            detailed_state = LiftDetailedState::tower;
        break;
    case LiftDetailedState::prep_place:
        detailed_state = LiftDetailedState::mid;
        break;

    case LiftDetailedState::place:
        target_lift_position = lift_place_positions[target_place_position.get()];
        target_shoulder_position = shoulder_intake_liftonly_clearance + 5_deg;
        gripper_control.Output = 0_V;
        target_twist_position = 0.5_tr;
        if(fabs(shoulder_encoder_position.GetValueAsDouble() - (shoulder_intake_liftonly_clearance + 5_deg).value()) < 0.01)
            detailed_state = LiftDetailedState::mid;

        break;

    case LiftDetailedState::eject_coral:
        lateral_drive_handle.release();
        detailed_state = LiftDetailedState::place;
        break;

    case LiftDetailedState::prep_algae:
        detailed_state = LiftDetailedState::mid;
        break;

    case LiftDetailedState::algae:
        target_lift_position = lift_algae_positions[target_place_position.get()];
        target_shoulder_position = shoulder_intake_liftonly_clearance + 5_deg;
        gripper_control.Output = 0_V;
        target_twist_position = 0.5_tr;
        if(fabs(shoulder_encoder_position.GetValueAsDouble() - (shoulder_intake_liftonly_clearance + 5_deg).value()) < 0.01)
            detailed_state = LiftDetailedState::prep_algae;
        break;
    
    default:
        break;
    }
}

void Lift::handle_place() {
    switch (detailed_state)
    {
    case LiftDetailedState::mid:
        detailed_state = LiftDetailedState::prep_place;
        target_place_position.take_control(0, false);
        if(target_place_position.get() >= std::size(lift_place_positions))
            target_place_position.set(0, std::size(lift_place_positions) - 1);
        if(target_place_position.get() < 0)
            target_place_position.set(0, 0);
        break;
    case LiftDetailedState::prep_place:
        target_lift_position = lift_place_positions[target_place_position.get()];
        target_shoulder_position = shoulder_intake_liftonly_clearance + 5_deg;
        gripper_control.Output = 0_V;
        target_twist_position = 0.5_tr;
        if(fabs(lift_position.GetValueAsDouble() - lift_place_positions[target_place_position.get()].value()) < 0.01)
            detailed_state = LiftDetailedState::place;
        break;
    case LiftDetailedState::place:
        target_lift_position = lift_place_positions[target_place_position.get()];
        target_shoulder_position = shoulder_place_position;
        gripper_control.Output = 0_V;
        target_twist_position = 0.5_tr;
        if(fabs(shoulder_encoder_position.GetValueAsDouble() - shoulder_place_position.value()) < 0.008){
            detailed_state = LiftDetailedState::eject_coral;
            eject_start = std::chrono::steady_clock::now();
        }
        break;
    case LiftDetailedState::eject_coral:
        target_lift_position = lift_place_positions[target_place_position.get()];
        target_shoulder_position = shoulder_place_position;
        gripper_control.Output = -4_V;
        target_twist_position = 0.5_tr;
        //skew backwards
        lateral_drive_handle.try_take_control();
        lateral_drive_handle.set(LateralSwerveRequest(-0.55_mps, 0_mps, SwerveRequestType::full_robot_relative));
        if(!gripper_coral && std::chrono::steady_clock::now() - eject_start > std::chrono::milliseconds(500)) 
        {
            target_mechanism_state.take_control(0, false);
            target_mechanism_state.set(0, LiftMechanismState::mid);
        }

        //check if the coral is out
        //if its out, cancel the movement
        break;
    default:
        break;
    }
}

void Lift::handle_algae() {
    switch (detailed_state)
    {
    case LiftDetailedState::mid:
        detailed_state = LiftDetailedState::prep_algae;
        target_place_position.take_control(0, false);
        if(target_place_position.get() >= std::size(lift_place_positions))
            target_place_position.set(0, std::size(lift_place_positions) - 1);
        if(target_place_position.get() < 0)
            target_place_position.set(0, 0);
        break;
    case LiftDetailedState::prep_algae:
        target_lift_position = lift_algae_positions[target_place_position.get()];
        target_shoulder_position = shoulder_intake_liftonly_clearance + 0_deg;
        gripper_control.Output = -5_V;
        target_twist_position = 0.25_tr;
        if(fabs(lift_position.GetValueAsDouble() - lift_algae_positions[target_place_position.get()].value()) < 0.01)
            detailed_state=LiftDetailedState::algae;
        break;

    case LiftDetailedState::algae:
        target_lift_position = lift_algae_positions[target_place_position.get()];
        target_shoulder_position = shoulder_algae_position;
        gripper_control.Output = -5_V;
        target_twist_position = 0.25_tr;
        break;
    
    default:
        break;
    }
}

void Lift::call(bool robot_enabled, bool autonomous) {

    ctre::phoenix6::BaseStatusSignal::RefreshAll(shoulder_motor_position, lift_position, shoulder_encoder_position);
    get_out_handle.try_take_control();

    //target_shoulder_position = 0.1_tr;

    //target_lift_position = 0.62_tr;

    frc::SmartDashboard::PutNumber("lift_detailed_state", (int)detailed_state);

    frc::SmartDashboard::PutNumber("coral_place_level", target_place_position.get() + 1);

    switch (target_mechanism_state.get())
    {
    case LiftMechanismState::park:
        handle_park();
        break;
    case LiftMechanismState::place:
        handle_place();
        break;
    case LiftMechanismState::pick:
        handle_pick();
        break;

    case LiftMechanismState::mid:
        handle_mid();
        break;

    case LiftMechanismState::algae:
        handle_algae();
        break;
    
    default:
        break;
    }

    if(!shoulder_motor.IsConnected())
        fault_manager.add_fault(Fault(true, FaultIdentifier::shoulderUnreachable));
    else 
        fault_manager.clear_fault(Fault(true, FaultIdentifier::shoulderUnreachable));
    
    if(!lift_motor.IsConnected())
        fault_manager.add_fault(Fault(true, FaultIdentifier::liftUnreachable));
    else 
        fault_manager.clear_fault(Fault(true, FaultIdentifier::liftUnreachable));

    if(!twist_motor.IsConnected())
        fault_manager.add_fault(Fault(true, FaultIdentifier::twistUnreachable));
    else 
        fault_manager.clear_fault(Fault(true, FaultIdentifier::twistUnreachable));


    //decide whether we want the intake out of the way
    //shoulder is out beyond the intake clearance and lift is below the intake(or the mechanism wants to pick)
    if((target_shoulder_position > shoulder_intake_liftonly_clearance && 
        (lift_position.GetValue() < lift_intake_liftonly_clearance || target_lift_position < lift_intake_liftonly_clearance)) ||
        target_mechanism_state.get() == LiftMechanismState::pick || shoulder_encoder_position.GetValue() > shoulder_level_position + 5_deg) {
        get_out_handle.set(intake::IntakeClearanceLevel::bay);
    }
    else if( //if lift is in collide zone, or lift target > liftonly clearance and lift < liftonly clearance, or lift target < liftonly clearance and lift > liftonly clearance
        target_lift_position > lift_park_position ||
        target_shoulder_position >= shoulder_clear_position ||
        lift_position.GetValue() > lift_park_position + 2.5_deg ||
        shoulder_encoder_position.GetValue() > shoulder_park_position + 8_deg)
    {
        get_out_handle.set(intake::IntakeClearanceLevel::lift_only);
    } else {
        get_out_handle.set(intake::IntakeClearanceLevel::none);
    }

    twist_motor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_twist_position));

    gripper_motor.SetControl(gripper_control);

    if(gripper_sensor.get_measurement().has_value()) {
        frc::SmartDashboard::PutNumber("gripper_distance", gripper_sensor.get_measurement().value().distance_mm);
        frc::SmartDashboard::PutNumber("gripper_ambient", gripper_sensor.get_measurement().value().ambient);
        gripper_coral = (gripper_sensor.get_measurement().value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT &&
            gripper_sensor.get_measurement().value().ambient < 32 && gripper_sensor.get_measurement().value().distance_mm < gripper_coral_threshold);
        frc::SmartDashboard::PutBoolean("gripper_coral", gripper_coral);
    } else if(HAL_GetRuntimeType() == HAL_RuntimeType::HAL_Runtime_Simulation) {
        frc::SmartDashboard::PutNumber("gripper_distance", 76);
        frc::SmartDashboard::PutNumber("gripper_ambient", 135);
        frc::SmartDashboard::PutBoolean("gripper_coral", true);
    }

    lift_control.Position = filter_lift_position(target_lift_position);
    lift_motor.SetControl(lift_control);

    switch (rotate_calibration_state)
    {
    case ShoulderCalibrationState::uncalibrated:
        if(!robot_enabled) break;
        shoulder_motor.SetControl(ctre::phoenix6::controls::VoltageOut(-0.19_V));
        rotate_calibration_start = std::chrono::steady_clock::now();
        rotate_calibration_state = ShoulderCalibrationState::calibrating;
        break;
    
    case ShoulderCalibrationState::calibrating:
        shoulder_motor.SetControl(ctre::phoenix6::controls::VoltageOut(-0.19_V));
        //fail if the calibration doesnt satisfy after 3 seconds
        if(std::chrono::steady_clock::now() - rotate_calibration_start > std::chrono::seconds(3)) {
            fault_manager.add_fault(Fault(false, FaultIdentifier::intakeRotateCalibrationTimeout));
            rotate_calibration_state = ShoulderCalibrationState::failed;
            shoulder_motor.Disable();
        }
        //must be on-time, on position, and stopped
        if(std::chrono::steady_clock::now() - rotate_calibration_start > std::chrono::milliseconds(1800) &&
            abs(shoulder_encoder.GetVelocity().GetValueAsDouble()) < 0.08 &&
            shoulder_encoder_position.GetValue() < shoulder_park_position + 8_deg) 
        {
            shoulder_motor.SetPosition(shoulder_encoder_position.GetValue() + 0.012_tr);
            rotate_calibration_state = ShoulderCalibrationState::calibrated;
            printf("Calibrated shoulder\n");
        }
        break;
    
    case ShoulderCalibrationState::calibrated:
        shoulder_control.Position = filter_shoulder_position(target_shoulder_position);
        shoulder_motor.SetControl(shoulder_control);
        break;
    }

    ui_table.get()->PutNumber("shoulder/calibration_state", (int)rotate_calibration_state);
    ui_table.get()->PutNumber("shoulder/target_position", shoulder_control.Position.value());
    ui_table.get()->PutNumber("shoulder/encoder_position", shoulder_encoder_position.GetValueAsDouble());
    ui_table.get()->PutNumber("lift/position", lift_position.GetValueAsDouble());
    ui_table.get()->PutNumber("lift/target_position", lift_control.Position.value());
    
    fault_manager.feed_watchdog();
}

bool Lift::is_paused() {
    return false;
}

void Lift::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::milliseconds(10);
}

Lift::~Lift() {

}