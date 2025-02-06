#include "OperatorInput.h"

void OperatorInput::call(bool robot_enabled, bool autonomous) {
    if(autonomous) return;

    if(js.GetRawButton(1) && js.GetRawButton(3)) { //square and circle
        tracking->set_gyro_angle(0_rad);
        tracking->set_camera_orientation_enabled(false);
    }
}

void OperatorInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(9000);
}

bool OperatorInput::is_paused() {
    return false;
}