#include "GyroInput.h"

#include <frc/Joystick.h>
#include <iostream>
#include "AHRS.h"
#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>

GyroInput::GyroInput(/* args */)
{

    //mxp->ZeroYaw();
    //mxp->SetAngleAdjustment()
    test_motor = new ctre::phoenix6::hardware::TalonFX(40);
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 1) {
        std::cout << "Error creating socket" << std::endl;
    }
    
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(6000);
    server_address.sin_addr.s_addr = INADDR_ANY;

    bind(sockfd, (struct sockaddr*)&server_address, sizeof(server_address));
}

void GyroInput::call(bool robot_enabled, bool autonomous) {
    //output.Set(js.GetRawButton(1));
    if(js.GetRawButton(1)) {
        //mxp->ZeroYaw();
    }

    //frc::SmartDashboard::PutNumber("yaw_rotation", mxp->GetRotation2d().Degrees().value());
    //frc::SmartDashboard::PutNumber("yaw_rate", mxp->GetRate());
    
    //if(mxp->IsCalibrating())
        return;
    
    //if(mxp->GetUpdateCount() == last_update_count)
        return;

    last_update_timestamp = std::chrono::steady_clock::now();
    //last_update_count = mxp->GetUpdateCount();
    //this->last_rotation = mxp->GetRotation2d();
    new_data = true;
}

frc::Rotation2d GyroInput::get_last_rotation() {
    return this->last_rotation;
}

std::chrono::time_point<std::chrono::steady_clock> GyroInput::get_timestamp() {
    return last_update_timestamp;
}

bool GyroInput::get_new() {
    bool ret = new_data;
    new_data = false;
    return ret;
}

void GyroInput::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(2000);
}

bool GyroInput::is_paused() {
    return false;
}

GyroInput::~GyroInput()
{
}