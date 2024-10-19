#pragma once
#include "Task.h"

#include "NoteHandler.h"
#include "Tracking.h"

#include "ControlChannel.h"

#define CALIBRATION_MAP_SIZE 5

enum class AutoShotMode {
    none,
    shoot
};

class AutoShot : public Task
{
private:
    controlchannel::ControlHandle<LauncherMode> launcher_mode_handle;

    controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle;
    controlchannel::ControlHandle<units::angular_velocity::turns_per_second_t> launcher_velocity_handle;

    Tracking *tracking;
    NoteHandler *note_handler;

    units::length::meter_t calibration_distances[CALIBRATION_MAP_SIZE] = {
        1.8_m,
        2.0_m,
        2.5_m,
        2.8_m,
        3.1_m};
    units::angle::radian_t calibration_angles[CALIBRATION_MAP_SIZE] = {
        0.75_rad,
        0.65_rad,
        0.22_rad,
        0.11_rad,
        0.11_rad
    };

    units::angular_velocity::turns_per_second_t calibration_speeds[CALIBRATION_MAP_SIZE] = {
        50_tps,
        55_tps,
        65_tps,
        75_tps,
        85_tps
    };

    units::angle::radian_t locked_angle;
    units::angular_velocity::turns_per_second_t locked_speed;

public:
    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    controlchannel::ControlChannel<AutoShotMode> mode_channel = controlchannel::ControlChannel<AutoShotMode>(AutoShotMode::none);

    AutoShot(
        controlchannel::ControlHandle<LauncherMode> launcher_mode_handle,
        controlchannel::ControlHandle<units::angle::radian_t> launcher_tilt_handle,
        controlchannel::ControlHandle<units::angular_velocity::turns_per_second_t> launcher_velocity_handle,
        Tracking *tracking,
        NoteHandler *note_handler): launcher_mode_handle(launcher_mode_handle), 
        launcher_tilt_handle(launcher_tilt_handle), tracking(tracking), 
        note_handler(note_handler), launcher_velocity_handle(launcher_velocity_handle) {};
    ~AutoShot() {};
};