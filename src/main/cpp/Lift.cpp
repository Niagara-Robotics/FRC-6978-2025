#include "Lift.h"

Lift::Lift(intake::Intake *intake): intake(intake),get_out_handle(intake->get_outta_the_way_channel.get_handle()) {
    lift_motor.GetConfigurator().Apply(lift_config);
    shoulder_motor.GetConfigurator().Apply(shoulder_config);
    shoulder_encoder.GetConfigurator().Apply(shoulder_encoder_configuration);

    shoulder_motor_position.SetUpdateFrequency(100_Hz);
    shoulder_encoder_position.SetUpdateFrequency(100_Hz);
    lift_position.SetUpdateFrequency(100_Hz);

    shoulder_motor.SetPosition(shoulder_encoder.GetAbsolutePosition().GetValue());

    lift_motor.SetPosition(0_tr);

    lift_control.UpdateFreqHz = 0_Hz;
}

units::angle::turn_t Lift::filter_lift_position(units::angle::turn_t input) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(shoulder_motor_position, lift_position, shoulder_encoder_position);
    input = (input > max_lift_position)? max_lift_position: input;
    input = (input < lift_park_position)? lift_park_position: input;
    //shoulder is in the bay traverse zone
    if(shoulder_encoder_position.GetValue() > shoulder_bay_enter_limit && shoulder_encoder_position.GetValue() < shoulder_bay_exit_limit)
        input = (input < lift_min_bay_traverse_position)? lift_min_bay_traverse_position : input;
    //shoulder is in the bay
    else if(shoulder_encoder_position.GetValue() > shoulder_bay_enter_limit) 
        input = (input < lift_min_bay_position)? lift_min_bay_position : input;
    else //shoulder is NOT in the bay
        input = (shoulder_encoder_position.GetValue() > shoulder_clear_position || !intake->is_clear_lift())? input : lift_park_position;



    return input;
}

units::angle::turn_t Lift::filter_shoulder_position(units::angle::turn_t input) {

}

void Lift::call(bool robot_enabled, bool autonomous) {

    ctre::phoenix6::BaseStatusSignal::RefreshAll(shoulder_motor_position, lift_position, shoulder_encoder_position);
    get_out_handle.try_take_control();

    //decide whether we want the intake out of the way
    get_out_handle.set(
        target_lift_position > lift_park_position ||
        target_shoulder_position >= shoulder_clear_position ||
        lift_position.GetValue() > lift_park_position + 2.5_deg ||
        shoulder_encoder_position.GetValue() > shoulder_park_position + 8_deg
    );



    lift_control.Position = filter_lift_position(target_lift_position);
    lift_motor.SetControl(lift_control);

    shoulder_control.Position = filter_shoulder_position(target_shoulder_position);
    //shoulder_motor.SetControl(shoulder_control);
}

bool Lift::is_paused() {
    return false;
}

void Lift::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::milliseconds(10);
}

Lift::~Lift() {

}