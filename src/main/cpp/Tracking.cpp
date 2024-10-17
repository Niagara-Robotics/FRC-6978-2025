#include "Tracking.h"

#include <frc/smartdashboard/SmartDashboard.h>


Tracking::Tracking(SwerveController *swerve_controller): 
    swerve_controller(swerve_controller)
{
    swerve_odometry = new frc::SwerveDriveOdometry<4>(swerve_controller->get_kinematics(), frc::Rotation2d(), swerve_controller->fetch_module_positions());
    odometry_pose_publisher = nt::StructTopic<frc::Pose2d>(nt::GetTopic(nt::GetDefaultInstance(), "tracking/odometry_pose")).Publish();
    mxp = new AHRS(frc::SPI::Port::kMXP, 1000000, 254);
    mxp->ZeroYaw();

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = POSE_STREAMER_PORT;

    if (bind(sockfd, (const struct sockaddr*) &servaddr, sizeof(servaddr)) < 0) {
        printf("Failed to bind socket\n");
    }
}

void Tracking::update_gyro() {
    std::chrono::duration<double, std::ratio<1, 1>> delta_t = std::chrono::steady_clock::now() - mxp_update_timestamp;
    mxp_update_timestamp = std::chrono::steady_clock::now();
    last_mxp_update_count = mxp->GetUpdateCount();

    gyro_rate = (mxp->GetRotation2d() - recent_gyro_pose) / delta_t.count();

    recent_gyro_pose = mxp->GetRotation2d();
}

void Tracking::set_gyro_angle(frc::Rotation2d rotation) {
    gyro_offset = rotation - recent_gyro_pose;
}

void Tracking::handle_packet(char buf[256]) {
    if(buf[0] != 0x33) return;

    if(buf[1] == 0x00) {
        //update heartbeat list
        return;
    }
    
    if(buf[1] == 0x01) {
        //send heartbeat
        return;
    }

    int object_count = buf[2];
    int offset = 0;

    double pitch, roll, yaw;
    double x, y, z;
    bool hasCamera;
    bool hasRotation;
    double distanceUsed = -1;

    for(int obj = 0; obj < object_count; obj++) {
        if(buf[3+offset] == 0x10 && buf[3+offset+1] == 0x00) { //camera observed robot position
            if(buf[3+offset+2] == 0x12) { //3d position from camera
                memcpy(&x, buf+3+offset+3, sizeof(double));
                memcpy(&y, buf+3+offset+3+sizeof(double), sizeof(double));
                memcpy(&z, buf+3+offset+3+(sizeof(double)*2), sizeof(double));
                offset+=24;
                hasCamera = true;
            } else if (buf[3+offset+2] == 0x11) { //2d rotation
                memcpy(&yaw, buf+3+offset+3, sizeof(double));
                offset+=8;
                hasRotation = true;
            }
        } else if (buf[3+offset] == 0x10 && buf[3+offset+1] == 0x03) {
            if(buf[3+offset+2] == 0x04) {
                memcpy(&distanceUsed, buf+3+offset+3, sizeof(double));
                offset+=8;
            }
        }
        offset += 3;
    }

    if(hasCamera && hasRotation) {
        //check distance threshold
        if(distanceUsed > 2.5) return;
        //check z threshold
        if(z > 1.5) return;
        //discriminate based on discrepancies in yaw angle
        if(!gyro_degraded && fabs(yaw - current_rotation_estimate.Radians().value()) > 0.01) return;

        //FIXME: update position
        swerve_odometry->ResetPosition(frc::Rotation2d(current_rotation_estimate), swerve_controller->fetch_module_positions(), frc::Pose2d(frc::Translation2d(x * 1.0_mm, y * 1.0_mm), frc::Rotation2d(current_rotation_estimate)));
        //check and update orientation
        if(distanceUsed < 1.5) {
            //update orientation
            set_gyro_angle(yaw * 1.0_rad);
        }
    }
}

void Tracking::call(bool robot_enabled, bool autonomous) {
    if(mxp->GetUpdateCount() != last_mxp_update_count) {
        update_gyro();
    }

    char buf[256];
    if(recv(sockfd, buf, 256, MSG_DONTWAIT) > 0) {
        
    }

    std::chrono::duration<double, std::ratio<1, 1>> gyro_delta = std::chrono::steady_clock::now() - mxp_update_timestamp;
    current_rotation_estimate = recent_gyro_pose + gyro_offset + (gyro_rate * gyro_delta.count()) * 1.0;

    swerve_odometry->Update(current_rotation_estimate, swerve_controller->fetch_module_positions());
    odometry_pose_publisher.Set(swerve_odometry->GetPose());
    swerve_controller->set_chassis_rotation(swerve_odometry->GetPose().Rotation());
}

frc::Pose2d Tracking::get_pose() {
    return swerve_odometry->GetPose();
}

void Tracking::reset() {
    swerve_odometry->ResetPosition(frc::Rotation2d(), swerve_controller->fetch_module_positions(), frc::Pose2d());
}

void Tracking::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(1250);
}

bool Tracking::is_paused() {
    return false;
}

Tracking::~Tracking() {

}