#include "Tracking.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SPI.h>
#include <iostream>
#include <arpa/inet.h>

Tracking::Tracking(SwerveController *swerve_controller): 
    swerve_controller(swerve_controller)
{
    swerve_odometry = new frc::SwerveDriveOdometry<4>(swerve_controller->get_kinematics(), frc::Rotation2d(), swerve_controller->fetch_module_positions());
    odometry_pose_publisher = nt::StructTopic<frc::Pose2d>(nt::GetTopic(nt::GetDefaultInstance(), "tracking/odometry_pose")).Publish();
    mxp = new studica::AHRS(studica::AHRS::kMXP_SPI, 200);
    mxp->ZeroYaw();

    frc::SmartDashboard::PutBoolean("data_from_vision", false);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(POSE_STREAMER_PORT);
    frc::SmartDashboard::PutNumber("socket_bind_failure", 0.0);
    if (bind(sockfd, (const struct sockaddr*) &servaddr, sizeof(servaddr)) < 0) {
        printf("Failed to bind socket\n");
        frc::SmartDashboard::PutNumber("socket_bind_failure", 1.0);
    }
    last_speaker_report = SpeakerReport(std::chrono::steady_clock::now(), 0.0_m, 0_rad);
}

void Tracking::update_gyro() {
    std::chrono::duration<double, std::ratio<1, 1>> delta_t = std::chrono::steady_clock::now() - mxp_last_update_local_timestamp;
    mxp_last_update_local_timestamp = std::chrono::steady_clock::now();
    last_mxp_timestamp = mxp->GetLastSensorTimestamp();

    gyro_rate = (mxp->GetRotation2d() - recent_gyro_pose) / delta_t.count();

    recent_gyro_pose = mxp->GetRotation2d();
}

void Tracking::update_orientation_estimate() {
    if(mxp->GetLastSensorTimestamp() != last_mxp_timestamp) {
        update_gyro();
    }

    std::chrono::duration<double, std::ratio<1, 1>> gyro_delta = std::chrono::steady_clock::now() - mxp_last_update_local_timestamp;
    current_rotation_estimate = recent_gyro_pose;
    current_rotation_estimate = current_rotation_estimate.RotateBy(gyro_offset + (gyro_rate * gyro_delta.count()) * 1.0);
}

void Tracking::set_gyro_angle(frc::Rotation2d rotation) {
    gyro_offset = rotation - recent_gyro_pose;
}

void Tracking::handle_packet(char buf[256]) {
    if(buf[0] != 0x33) return;
    
    if(buf[1] == 0x01) {
        //send heartbeat
        //printf("received heartbeat\n");
        last_received_heartbeat = std::chrono::steady_clock::now();
        last_network_latency = std::chrono::duration_cast<std::chrono::milliseconds>(last_received_heartbeat - last_sent_heartbeat);
        frc::SmartDashboard::PutNumber("Vision Latency", std::chrono::duration_cast<std::chrono::microseconds>(last_received_heartbeat - last_sent_heartbeat).count());
        return;
    }

    int object_count = buf[2];
    int offset = 0;

    double pitch, roll, yaw;
    double x, y, z;
    bool hasCamera;
    bool hasRotation;
    double distanceUsed = -1;
    int32_t tag_id = -1;
    double tag_heading;
    int latency = 0;

    for(int obj = 0; obj < object_count; obj++) {
        if(buf[3+offset] == PS_OCI_CLASS_TRACKING && buf[3+offset+1] == PS_OCI_SUBCLASS_ROBOT_CAMERA_POSE) { //camera observed robot position
            if(buf[3+offset+2] == PS_OTI_3D_POSITION) { //3d position from camera
                memcpy(&x, buf+3+offset+3, sizeof(double));
                memcpy(&y, buf+3+offset+3+sizeof(double), sizeof(double));
                memcpy(&z, buf+3+offset+3+(sizeof(double)*2), sizeof(double));
                offset+=sizeof(double)*3;
                hasCamera = true;
            } else if (buf[3+offset+2] == PS_OTI_2D_ROTATION) { //2d rotation
                memcpy(&yaw, buf+3+offset+3, sizeof(double));
                offset+=8;
                hasRotation = true;
            }
        } else if (buf[3+offset] == PS_OCI_CLASS_TRACKING && buf[3+offset+1] == PS_OCI_SUBCLASS_TAG_DISTANCE) {
            if(buf[3+offset+2] == PS_OTI_DOUBLE) {
                memcpy(&distanceUsed, buf+3+offset+3, sizeof(double));
                offset+=8;
            }
        } else if (buf[3+offset] == PS_OCI_CLASS_TRACKING && buf[3+offset+1] == PS_OCI_SUBCLASS_TAG_ID) { //tag ID
            if(buf[3+offset+2] == PS_OTI_INTEGER) { //integer
                memcpy(&tag_id, buf+3+offset+3, sizeof(int32_t));
                offset+=4;
            }
        } else if (buf[3+offset] == PS_OCI_CLASS_TRACKING && buf[3+offset+1] == PS_OCI_SUBCLASS_TAG_HEADING) { //camera observed tag heading
            if(buf[3+offset+2] == PS_OTI_DOUBLE) {
                memcpy(&tag_heading, buf+3+offset+3, sizeof(double));
                offset+=8;
            }
        } else if (buf[3+offset] == PS_OCI_CLASS_TRACKING && buf[3+offset+1] == PS_OCI_SUBCLASS_LATENCY) { //camera observed tag heading
            if(buf[3+offset+2] == PS_OTI_INTEGER) {
                memcpy(&latency, buf+3+offset+3, sizeof(int32_t));
                offset+=4;
            }
        }
        offset += 3;
    }

    yaw = yaw - M_PI;

    auto alliance = frc::DriverStation::GetAlliance();
    distanceUsed /= 1000.0;
    frc::SmartDashboard::PutBoolean("data_from_vision", true);
    frc::SmartDashboard::PutNumber("Vision Processing Time", latency);
    frc::SmartDashboard::PutNumber("Vision Composite Time", latency+last_network_latency.count());
    
    if(hasCamera && hasRotation) {
        //printf("successfully decoded packet\n");
        //check distance threshold
        if(distanceUsed > 2.8) return;
        if(!camera_orientation_enabled) return;

        //check z threshold
        //if(z > 1.5) return;

        //discriminate based on discrepancies in yaw angle        
        //if(!gyro_degraded && fabs(yaw - current_rotation_estimate.Radians().value()) > 0.07) return;
        last_camera_pose_update = std::chrono::steady_clock::now();
        //swerve_odometry->ResetPosition(frc::Rotation2d(current_rotation_estimate), swerve_controller->fetch_module_positions(), frc::Pose2d(frc::Translation2d(x * 1.0_mm, y * 1.0_mm), frc::Rotation2d(current_rotation_estimate)));
        //check and update orientation
        if(distanceUsed < 1.9 && camera_orientation_enabled) {
            //update orientation
            //set_gyro_angle(yaw * 1.0_rad);
            gyro_degraded = false;
            last_gyro_camera_sync = std::chrono::steady_clock::now();
            push_camera_update(true, frc::Pose2d(frc::Translation2d(x*1.0_mm, y*1.0_mm), frc::Rotation2d(yaw*1.0_rad)), std::chrono::steady_clock::now() - (std::chrono::milliseconds(latency)+last_network_latency));

        } else {
            push_camera_update(false, frc::Pose2d(frc::Translation2d(x*1.0_mm, y*1.0_mm), frc::Rotation2d(yaw*1.0_rad)), std::chrono::steady_clock::now() - (std::chrono::milliseconds(latency)+last_network_latency));
        }
    }
}

void Tracking::send_heartbeat() {
    char outbuf[32];
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, "10.69.78.3", &servaddr.sin_addr);
    servaddr.sin_port = htons(POSE_STREAMER_PORT);
    
    outbuf[0] = 32;
    sendto(sockfd, outbuf, 32, MSG_DONTWAIT, (const struct sockaddr*) &servaddr, sizeof(servaddr));
    last_sent_heartbeat = std::chrono::steady_clock::now();
}

void Tracking::set_camera_orientation_enabled(bool enabled) {
    camera_orientation_enabled = enabled;
}

SpeakerReport Tracking::get_speaker_pose() {
    return last_speaker_report;
}

void Tracking::push_camera_update(bool affect_rotation, frc::Pose2d camera_pose, std::chrono::steady_clock::time_point exposure_timestamp) {
    if(tracking_buffer.size() < 1) return;
    TrackingFrame odometry_frame = tracking_buffer.front();
    tracking_buffer.pop_front();
    int i = 1;
    while (odometry_frame.observation_time < exposure_timestamp) //until the odometry frame comes after the camera frame
    {
        if(tracking_buffer.size() < 1) break;
        odometry_frame = tracking_buffer.front();
        tracking_buffer.pop_front();
        i++;
    }
    //printf("Fast-forwarding %i odometry frames\n", tracking_buffer.size());
    //set the offset
    
    frc::Rotation2d temp_gyro_offset = camera_pose.Rotation() - odometry_frame.gyro_rotation;

    if(affect_rotation)
        swerve_odometry->ResetPosition(odometry_frame.gyro_rotation + temp_gyro_offset, odometry_frame.module_positions, camera_pose);
    else 
        swerve_odometry->ResetPosition(odometry_frame.rotation_estimate, odometry_frame.module_positions, frc::Pose2d(camera_pose.Translation(), odometry_frame.rotation_estimate));
    while(tracking_buffer.size() > 0) {
        odometry_frame = tracking_buffer.front();
        tracking_buffer.pop_front();
        
        if(affect_rotation)
            swerve_odometry->Update(odometry_frame.gyro_rotation + temp_gyro_offset, odometry_frame.module_positions);
        else 
            swerve_odometry->Update(odometry_frame.rotation_estimate, odometry_frame.module_positions);
    }
    if(affect_rotation) gyro_offset = temp_gyro_offset;
}

void Tracking::call(bool robot_enabled, bool autonomous) {
    update_orientation_estimate();

    //printf("handling packets\n");

    char buf[256];
    if(recv(sockfd, buf, 256, MSG_DONTWAIT) > 0) {
        handle_packet(buf);
    }

    send_heartbeat();

    wpi::array<frc::SwerveModulePosition, 4> module_positions = swerve_controller->fetch_module_positions();

    swerve_odometry->Update(current_rotation_estimate, module_positions);
    //printf("updateing trakcing buffer\n");
    tracking_buffer.push_back(TrackingFrame(current_rotation_estimate, recent_gyro_pose, swerve_odometry->GetPose(), module_positions));
    if(tracking_buffer.size() > TRACKING_BUFFER_CAP) {
        tracking_buffer.pop_front();
    }

    odometry_pose_publisher.Set(swerve_odometry->GetPose());
    swerve_controller->set_chassis_rotation(swerve_odometry->GetPose().Rotation());

    frc::SmartDashboard::PutBoolean("gyro_not_degraded", !gyro_degraded);
    frc::SmartDashboard::PutBoolean("camera_orientation_enabled", camera_orientation_enabled);
    frc::SmartDashboard::PutNumber("gyro_angle", mxp->GetAngle());
}

frc::Pose2d Tracking::get_pose() {
    return swerve_odometry->GetPose();
}

frc::ChassisSpeeds Tracking::get_chassis_speeds() {
    return swerve_controller->get_chassis_speeds();
}

void Tracking::reset() {
    swerve_odometry->ResetPosition(frc::Rotation2d(), swerve_controller->fetch_module_positions(), frc::Pose2d());
}

void Tracking::reset_pose(frc::Pose2d pose) {
    set_gyro_angle(pose.Rotation());
    swerve_odometry->ResetPosition(pose.Rotation(), swerve_controller->fetch_module_positions(), pose);
}

void Tracking::schedule_next(std::chrono::time_point<std::chrono::steady_clock> current_time) {
    this->next_execution = current_time + std::chrono::microseconds(1700);
}

bool Tracking::is_paused() {
    return false;
}

Tracking::~Tracking() {

}