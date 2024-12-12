#include "AutoShot.h"

#include <frc/smartdashboard/SmartDashboard.h>

void AutoShot::call(bool robot_enabled, bool autonomous) {
    
    switch (mode_channel.get())
    {
    case AutoShotMode::shoot:
        
        launcher_tilt_handle.try_take_control();
        launcher_tilt_handle.set(locked_angle);
        launcher_mode_handle.try_take_control();
        launcher_mode_handle.set(LauncherMode::tilt_interlock);
        launcher_velocity_handle.try_take_control();
        launcher_velocity_handle.set(locked_speed);
        if(note_handler->get_fire_timer_elapsed()) {
            mode_channel.take_control(0,true);
            launcher_tilt_handle.set(0.05_rad);
            mode_channel.set(0, AutoShotMode::none);
            launcher_mode_handle.set(LauncherMode::idle);
        }
        break;
    
    default: //do nothing
        launcher_mode_handle.set(LauncherMode::idle);
        launcher_tilt_handle.set(0.05_rad);
        SpeakerReport speaker_pose = tracking->get_speaker_pose();
        int upper_index = 0;
        for (upper_index=0; upper_index < CALIBRATION_MAP_SIZE; upper_index++) {
            if(speaker_pose.distance < calibration_distances[upper_index]) break;
        }
        int lower_index = upper_index -1;

        double interpolation_factor = (speaker_pose.distance - calibration_distances[lower_index]) / (calibration_distances[upper_index] - calibration_distances[lower_index]);
        locked_angle = calibration_angles[lower_index] + ((calibration_angles[upper_index] - calibration_angles[lower_index]) * interpolation_factor);
        locked_angle += offset_channel.get();
        locked_speed = calibration_speeds[lower_index] + ((calibration_speeds[upper_index] - calibration_speeds[lower_index]) * interpolation_factor);

        if(speaker_pose.distance < calibration_distances[0]) {
            locked_angle = calibration_angles[0];
            locked_speed = calibration_speeds[0];
        }
        if(speaker_pose.distance > calibration_distances[CALIBRATION_MAP_SIZE-1]) {
            locked_angle = calibration_angles[CALIBRATION_MAP_SIZE-1];
            locked_speed = calibration_speeds[CALIBRATION_MAP_SIZE-1];
        }

        if(std::chrono::steady_clock::now() > (speaker_pose.observation_time + std::chrono::milliseconds(250))){
            frc::SmartDashboard::PutString("autoshot_distance_status", "no tag");
            frc::SmartDashboard::PutBoolean("autoshot_distance_okay", false);
        } else if(speaker_pose.distance < min_safe_distance) {
            frc::SmartDashboard::PutString("autoshot_distance_status", "too close");
            frc::SmartDashboard::PutBoolean("autoshot_distance_okay", false);
        } else if(speaker_pose.distance > max_safe_distance) {
            frc::SmartDashboard::PutString("autoshot_distance_status", "too far");
            frc::SmartDashboard::PutBoolean("autoshot_distance_okay", false);
        } else {
            frc::SmartDashboard::PutString("autoshot_distance_status", "okay");
            frc::SmartDashboard::PutBoolean("autoshot_distance_okay", true);
        }

        if(locked_angle > 0.75_rad) locked_angle = 0.75_rad;
        if(locked_angle < 0.11_rad) locked_angle = 0.11_rad;
        frc::SmartDashboard::PutNumber("autoshot_angle", locked_angle.value());
        frc::SmartDashboard::PutNumber("autoshot_offset", offset_channel.get().value());
        break;
    }
}

void AutoShot::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(5000);
}

bool AutoShot::is_paused() {
    return false;
}