#include "DriverInput.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#define DEAD_ZONE 0.15
#define xyMultiplier 2.0_mps
#define wMultiplier 4.14_rad_per_s

#define BUTTON_TAKE_CONTROL 2

void DriverInput::call(bool robot_enabled, bool autonomous) {
    double x = -js.GetRawAxis(1); //joystick y is robot x
    double y = -js.GetRawAxis(0);
    double omega = -js.GetRawAxis(2);

    x = (fabs(x) > DEAD_ZONE)? 
            ((x > 0)? 
                ((x-DEAD_ZONE)/(1-DEAD_ZONE)) :
                ((x+DEAD_ZONE)/(1-DEAD_ZONE))
            ) 
            : 0;

    y = (fabs(y) > DEAD_ZONE)? 
        ((y > 0)? 
            ((y-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((y+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;

    omega = (fabs(omega) > DEAD_ZONE)? 
        ((omega> 0)? 
            ((omega-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((omega+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;

    if(js.GetRawButton(BUTTON_TAKE_CONTROL)) {
        //std::cout << "grabbing handles" << std::endl;
        if(!planar_handle.try_take_control()) {
            std::cout << "Failed to grab planar handle" << std::endl;
        }
        ap_twist_mode_handle.try_take_control();
        ap_twist_mode_handle.set(AutoPilotTwistMode::none);

        twist_handle.try_take_control();

    }
    //y = 0;
    planar_handle.set(PlanarSwerveRequest(x*xyMultiplier, y*xyMultiplier));
    twist_handle.set(omega * wMultiplier);

    if(js.GetRawButton(5)){
        tracking->reset();
    }

    if(js.GetRawButton(1)) {
        tracking->set_gyro_angle(0_rad);
    }

    if(js.GetRawButton(3)) {
        ap_twist_mode_handle.try_take_control();
        ap_heading_handle.try_take_control();
        ap_twist_mode_handle.set(AutoPilotTwistMode::heading);
    }
    if(js.GetRawButton(4)) {
        ap_twist_mode_handle.try_take_control();
        ap_heading_handle.try_take_control();
        ap_twist_mode_handle.set(AutoPilotTwistMode::face);
    }

    int pov = js.GetPOV(0);

    switch (pov)
    {
    case 0:
        ap_heading_handle.set(0_deg);
        break;
    case 270:
        ap_heading_handle.set(90_deg);
        break;
    case 90:
        ap_heading_handle.set(-90_deg);
        break;
    case 360:
        ap_heading_handle.set(180_deg);
        break;
    default:
        break;
    }

    if(js.GetRawButton(6)){
        index_mode_handle.try_take_control();
        index_mode_handle.set(IntakeIndexingMode::roll_in);
    } else {
        index_mode_handle.set(IntakeIndexingMode::stop);
    }
    
    if(js.GetRawButton(8)) {
        launcher_mode_handle.try_take_control();
        launcher_mode_handle.set(LauncherMode::velocity_interlock);
    } else {
        launcher_mode_handle.set(LauncherMode::idle);
    }

    if(js.GetRawButton(10)) {
        launcher_tilt_handle.try_take_control();
        launcher_tilt_handle.set(0.93_rad);
    } else if (js.GetRawButton(14)) {
        launcher_tilt_handle.set(0.05_rad);
    }

}

void DriverInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(9000);
}

bool DriverInput::is_paused() {
    return false;
}