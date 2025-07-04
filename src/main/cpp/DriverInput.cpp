#include "DriverInput.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#define DEAD_ZONE 0.07
#define xyMultiplier 4.0_mps
#define wMultiplier 4.5_rad_per_s

#define BUTTON_TAKE_CONTROL 2

void DriverInput::call(bool robot_enabled, bool autonomous) {
    auto alliance = frc::DriverStation::GetAlliance();

    double x,y,omega, radius, angle;
    const double linearity = 0.35;

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

    x = -js.GetRawAxis(1); //joystick y is robot x
    y = -js.GetRawAxis(0);
    omega = -js.GetRawAxis(2);

    radius = sqrt(pow(x, 2) + pow(y, 2));

    radius = (1 - linearity)*pow(radius, 3) + (linearity)*radius;
    angle = atan2(y, x);

    radius = (fabs(radius) > DEAD_ZONE)? 
            ((radius > 0)? 
                ((radius-DEAD_ZONE)/(1-DEAD_ZONE)) :
                ((radius+DEAD_ZONE)/(1-DEAD_ZONE))
            ) 
            : 0;

    frc::SmartDashboard::PutNumber("driver_radius", radius);
    frc::SmartDashboard::PutNumber("driver_angle", angle);

    x = cos(angle) * radius;
    y = sin(angle) * radius;

    frc::SmartDashboard::PutNumber("driver_x", x);
    frc::SmartDashboard::PutNumber("driver_y", y);

    /*y = (fabs(y) > DEAD_ZONE)? 
        ((y > 0)? 
            ((y-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((y+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;*/

    omega = (fabs(omega) > DEAD_ZONE)? 
        ((omega> 0)? 
            ((omega-DEAD_ZONE)/(1-DEAD_ZONE)) :
            ((omega+DEAD_ZONE)/(1-DEAD_ZONE))
        ) 
        : 0;

    
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed && !robot_relative) {
        x = -x;
        y = -y;
    }
    
    if(js.GetRawButton(BUTTON_TAKE_CONTROL)) {
        //std::cout << "grabbing handles" << std::endl;
        if(!planar_handle.try_take_control() || !twist_handle.try_take_control()) {
            std::cout << "Failed to grab planar handle" << std::endl;
            fault_manager.add_fault(Fault(false, FaultIdentifier::drivebaseTakeoverFailed));
        } else {
            fault_manager.clear_fault(Fault(false, FaultIdentifier::drivebaseTakeoverFailed));
        }

        ap_translate_handle.try_take_control();
        ap_translate_handle.set(AutoPilotTranslateMode::none);

        ap_twist_handle.try_take_control();
        ap_twist_handle.set(AutoPilotTwistMode::none);
    }

    planar_handle.set(LateralSwerveRequest(x*xyMultiplier, y*xyMultiplier, robot_relative? SwerveRequestType::full_robot_relative: SwerveRequestType::full));
    twist_handle.set(omega * wMultiplier);

    if(js.GetRawButton(10) && !last_rrel_button) {
        robot_relative = !robot_relative;
    }
    last_rrel_button = js.GetRawButton(10);


    if(js.GetRawButton(1)) {
        ap_translate_handle.try_take_control();
        ap_translate_handle.set(AutoPilotTranslateMode::reef);

        ap_twist_handle.try_take_control();
        ap_twist_handle.set(AutoPilotTwistMode::reef);
        
        ap_tree_handle.try_take_control();
        ap_tree_handle.set(ReefTree::left);
    }

    if(js.GetRawButton(3)) {
        ap_translate_handle.try_take_control();
        ap_translate_handle.set(AutoPilotTranslateMode::reef);

        ap_twist_handle.try_take_control();
        ap_twist_handle.set(AutoPilotTwistMode::reef);

        ap_tree_handle.try_take_control();
        ap_tree_handle.set(ReefTree::right);
    }

    frc::SmartDashboard::PutBoolean("input_robot_relative", robot_relative);

    

    if(js.GetRawButton(9)) {
        intake_handle.try_take_control();
        intake_handle.set(intake::IntakeAction::pickup_algae);
    }
    if(js.GetRawButton(5)) {
        intake_handle.try_take_control();
        intake_handle.set(intake::IntakeAction::eject_algae);
    }
    if(js.GetRawButton(4)) {
        intake_handle.try_take_control();
        intake_handle.set(intake::IntakeAction::standby);
    }
    if(js.GetRawButton(6)) {
        intake_handle.try_take_control();
        intake_handle.set(intake::IntakeAction::pickup_coral);
    }

    if(js.GetRawButton(8)) {
        lift_handle.try_take_control();
        lift_handle.set(LiftMechanismState::place);
        ap_translate_handle.set(AutoPilotTranslateMode::none);
    }


    watchdog:

    fault_manager.feed_watchdog();
}

void DriverInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(10000);
}

bool DriverInput::is_paused() {
    return false;
}