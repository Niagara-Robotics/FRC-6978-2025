#include "AutoPilot.h"

//close_shot

void AutoPilot::close_shot_init() {
    launcher_mode_handle.try_take_control();
    launcher_tilt_handle.try_take_control();

    launcher_tilt_handle.set(0.75_rad);
    launcher_mode_handle.set(LauncherMode::tilt_interlock);
}

void AutoPilot::close_shot_exec() {

}

void AutoPilot::close_shot_kill(bool interrupt) {
    launcher_tilt_handle.set(0.05_rad);
    launcher_mode_handle.set(LauncherMode::idle);
}

bool AutoPilot::close_shot_complete() {
    return note_handler->get_fire_timer_elapsed();
}

//pickup

void AutoPilot::pickup_init() {
    printf("Started pickup\n");
    index_mode_handle.try_take_control();
    index_mode_handle.set(IntakeIndexingMode::roll_in);
}

void AutoPilot::pickup_exec() {
    
}

void AutoPilot::pickup_kill(bool interrupt) {
    index_mode_handle.try_take_control();
    index_mode_handle.set(IntakeIndexingMode::stop);
    printf("killed pickup\n");
}

bool AutoPilot::pickup_complete() {
    return note_handler->get_index_state() == IntakeIndexingState::hold;
}

//auto_shot

void AutoPilot::auto_shot_init() {
    auto_shot_mode_handle.try_take_control();
    auto_shot_mode_handle.set(AutoShotMode::shoot);
}

void AutoPilot::auto_shot_exec() {
    auto_shot_mode_handle.try_take_control();
    auto_shot_mode_handle.set(AutoShotMode::shoot);
}

void AutoPilot::auto_shot_kill(bool interrupt) {
    auto_shot_mode_handle.set(AutoShotMode::none);
}

bool AutoPilot::auto_shot_complete() {
    return note_handler->get_fire_timer_elapsed();
}