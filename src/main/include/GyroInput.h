#pragma once
#include "Task.h"
#include <frc/Joystick.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "AHRS.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <chrono>

class GyroInput : public Task
{
private:
    frc::Joystick js = frc::Joystick(0);
    ctre::phoenix6::hardware::TalonFX *test_motor;
    frc::DigitalInput test_input = frc::DigitalInput(0);
    int sockfd;
    sockaddr_in server_address;
    
    frc::Rotation2d last_rotation;

    double last_update_count;

    std::chrono::time_point<std::chrono::steady_clock> last_update_timestamp;

    bool new_data;
public:
    void schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) override;
    void call(bool robot_enabled, bool autonomous) override;
    bool is_paused() override;

    frc::Rotation2d get_last_rotation();
    std::chrono::time_point<std::chrono::steady_clock> get_timestamp();
    bool get_new();

    GyroInput(/* args */);
    ~GyroInput();
};


