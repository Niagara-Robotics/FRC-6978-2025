#include "OperatorInput.h"

void OperatorInput::call(bool robot_enabled, bool autonomous) {
    if(autonomous) return;

    if(!js.IsConnected()) {
        fault_manager.add_fault(Fault(true, FaultIdentifier::driverControllerUnreachable));
        planar_handle.release();
        twist_handle.release();
        goto watchdog;
    }

    if(js.GetRawButton(1) && js.GetRawButton(3)) { //square and circle
        tracking->set_gyro_angle(0_rad);
        tracking->set_camera_orientation_enabled(false);
    }
    if(js.GetRawButton(2)) {
        lift_handle.try_take_control();
        lift_handle.set(LiftMechanismState::mid);
    }
    if(js.GetRawButton(5)) {
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
        default:
            break;
        }
    }
    last_pov = js.GetPOV();

    watchdog:
    fault_manager.feed_watchdog();
}

void OperatorInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(9000);
}

bool OperatorInput::is_paused() {
    return false;
}