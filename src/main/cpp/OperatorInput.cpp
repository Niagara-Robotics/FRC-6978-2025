#include "OperatorInput.h"

void OperatorInput::call(bool robot_enabled, bool autonomous) {
    if(autonomous) return;

    if(js.GetRawButton(1) && js.GetRawButton(3)) { //square and circle
        tracking->set_gyro_angle(0_rad);
        tracking->set_camera_orientation_enabled(false);
    }

    // move tilt to SDB preset

    if(js.GetRawButton(7)) { //left trigger
        launcher_tilt_handle.try_take_control();
        launcher_tilt_handle.set(frc::SmartDashboard::GetNumber("launcher_test_angle", 0.7)*1_rad);
    } else if (js.GetRawButton(14)) { //drop tilt
        launcher_tilt_handle.set(0.05_rad);
    }

    //fire the launcher(ignore tilt)

    if(js.GetRawButton(8)) { //right trigger
        launcher_mode_handle.try_take_control();
        launcher_mode_handle.set(LauncherMode::velocity_interlock);

        launcher_velocity_handle.try_take_control();
        launcher_velocity_handle.set(65_tps);
        
    } else {
        launcher_mode_handle.set(LauncherMode::idle);
    } 

    //disbable camera orientation
    if(js.GetRawButton(10)) //right system???????
    {

    }

    if(js.GetRawButton(4)) {
        ap_twist_mode_handle.try_take_control();
        ap_twist_mode_handle.set(AutoPilotTwistMode::speaker);
    } else {
        ap_twist_mode_handle.set(AutoPilotTwistMode::none);
    }

    if(js.GetPOV() != last_pov) {
        last_pov = js.GetPOV();
        auto_shot_offset_handle.try_take_control();
        if(last_pov== 0) {
            auto_shot_offset_handle.set(auto_shot_offset_handle.get() + 0.05_rad);
        }

        if(last_pov== 180) {
            auto_shot_offset_handle.set(auto_shot_offset_handle.get() - 0.05_rad);
        }
    }
}

void OperatorInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(9000);
}

bool OperatorInput::is_paused() {
    return false;
}