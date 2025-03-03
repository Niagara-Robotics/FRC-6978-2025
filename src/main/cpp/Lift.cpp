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

void Lift::call(bool robot_enabled, bool autonomous) {
    units::angle::turn_t actual_lift_target = target_lift_position;
    units::angle::turn_t actual_shoulder_target = target_shoulder_position;

    ctre::phoenix6::BaseStatusSignal::RefreshAll(shoulder_motor_position, lift_position, shoulder_encoder_position);
    get_out_handle.try_take_control();
    

    actual_lift_target = (actual_lift_target > max_lift_position)? max_lift_position: actual_lift_target;
    actual_lift_target = (actual_lift_target < lift_park_position)? lift_park_position: actual_lift_target;

    actual_lift_target = (shoulder_encoder_position.GetValue() > shoulder_clear_position || !intake->is_clear_lift())? actual_lift_target : lift_park_position;



    get_out_handle.set(
        actual_lift_target > lift_park_position ||
        actual_shoulder_target >= shoulder_clear_position ||
        lift_position.GetValue() > lift_park_position + 2.5_deg ||
        shoulder_encoder_position.GetValue() > shoulder_park_position + 8_deg
    );



    lift_control.Position = actual_lift_target;
    lift_motor.SetControl(lift_control);
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