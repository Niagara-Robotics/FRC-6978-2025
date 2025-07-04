#include "OperatorInput.h"

void OperatorInput::call(bool robot_enabled, bool autonomous) {

    if(!js.IsConnected()) {
        fault_manager.add_fault(Fault(true, FaultIdentifier::controllerUnreachable));
        planar_handle.release();
        twist_handle.release();
        goto watchdog;
    } else {
        fault_manager.clear_fault(Fault(true, FaultIdentifier::controllerUnreachable));
    }

    if(js.GetButtonCount() < 10 || js.GetAxisCount() < 3)
        fault_manager.add_fault(Fault(true, FaultIdentifier::incorrectController));
    else 
        fault_manager.clear_fault(Fault(true, FaultIdentifier::incorrectController));

    if(!robot_enabled || autonomous) goto watchdog;

    if(js.GetRawButton(1) && js.GetRawButton(3)) { //square and circle
        tracking->set_gyro_angle(0_rad);
        tracking->set_camera_orientation_enabled(false);
    }
    if(js.GetRawButton(2)) {
        lift_handle.try_take_control();
        lift_handle.set(LiftMechanismState::algae);
    }
    if(js.GetRawButton(5)) {
        lift_handle.try_take_control();
        lift_handle.set(LiftMechanismState::mid);
    }
    if(js.GetRawButton(4)) {
        lift_handle.try_take_control();
        lift_handle.set(LiftMechanismState::pick);
    }

    if(js.GetPOV() != last_pov) {
        switch (js.GetPOV())
        {
        case 0:
            if(!place_position_handle.try_take_control()) 
                break;
            place_position_handle.set(place_position_handle.get()+1);
            if(place_position_handle.get() > 3)
                place_position_handle.set(3);
            break;
        case 180:
            if(!place_position_handle.try_take_control()) 
                break;
            place_position_handle.set(place_position_handle.get()-1);
            if(place_position_handle.get() < 0)
                place_position_handle.set(0);

            break;
        case 90:
            tree_handle.try_take_control();
            tree_handle.set(ReefTree::right);
            break;
        case 270:
            tree_handle.try_take_control();
            tree_handle.set(ReefTree::left);
            break;
        default:
            break;
        }
    }
    last_pov = js.GetPOV();

    if(js.GetRawButton(9) != last_vision_mask_button) {
        if(js.GetRawButton(9)) {
            vision_mask_handle.try_take_control();
            vision_mask_handle.set(!vision_mask_handle.get());
        }
        last_vision_mask_button = js.GetRawButton(9);
    }

    if(js.GetRawButton(14)) {
        ap_translate_handle.try_take_control();
        ap_translate_handle.set(AutoPilotTranslateMode::reef);

        ap_twist_handle.try_take_control();
        ap_twist_handle.set(AutoPilotTwistMode::reef);
    }

    watchdog:
    fault_manager.feed_watchdog();
}

void OperatorInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(10000);
}

bool OperatorInput::is_paused() {
    return false;
}