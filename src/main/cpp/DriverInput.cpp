#include "DriverInput.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#define DEAD_ZONE 0.15
#define xyMultiplier 2.0_mps
#define wMultiplier 4.5_rad_per_s

#define BUTTON_TAKE_CONTROL 2

void DriverInput::call(bool robot_enabled, bool autonomous) {
    if(autonomous) return;
    
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

    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
        x = -x;
        y = -y;
    }
    
    if(js.GetRawButton(BUTTON_TAKE_CONTROL)) {
        //std::cout << "grabbing handles" << std::endl;
        if(!planar_handle.try_take_control()) {
            std::cout << "Failed to grab planar handle" << std::endl;
        }

        twist_handle.try_take_control();
    }

    planar_handle.set(PlanarSwerveRequest(x*xyMultiplier, y*xyMultiplier, robot_relative));
    twist_handle.set(omega * wMultiplier);

    if(js.GetRawButton(10) && !last_rrel_button) {
        robot_relative = !robot_relative;
    }
    last_rrel_button = js.GetRawButton(10);

    frc::SmartDashboard::PutBoolean("robot_relative", robot_relative);

}

void DriverInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(9000);
}

bool DriverInput::is_paused() {
    return false;
}