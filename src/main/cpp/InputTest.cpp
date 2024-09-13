#include "InputTest.h"

#include <frc/Joystick.h>
#include <iostream>
#include "AHRS.h"
#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>

InputTest::InputTest(/* args */)
{
    mxp = new AHRS(frc::SPI::Port::kMXP, 200);
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

void InputTest::call() {
    //output.Set(js.GetRawButton(1));
    if(js.GetRawButton(1)) {
        mxp->ZeroYaw();
    }
    frc::SmartDashboard::PutNumber("yaw_rotation", mxp->GetRotation2d().Degrees().value());
    frc::SmartDashboard::PutNumber("yaw_rate", mxp->GetRate());
    this->last_rotation = mxp->GetRotation2d();

    //start the motor
    
    test_input.Get();
    frc::SmartDashboard::PutBoolean("test sensor", !test_input.Get());
    if(!test_input.Get()) {
        test_motor->SetControl(ctre::phoenix6::controls::NeutralOut());
    } else {
        test_motor->SetControl(ctre::phoenix6::controls::VoltageOut(-0.9_V));
    }
    frc::SmartDashboard::PutNumber("velocity", test_motor->GetVelocity().GetValue().value());
}

frc::Rotation2d InputTest::get_last_rotation() {
    return this->last_rotation;
}

void InputTest::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(10000);
}

bool InputTest::is_paused() {
    return false;
}

InputTest::~InputTest()
{
}