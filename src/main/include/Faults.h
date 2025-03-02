#pragma once

enum class FaultIdentifier {
// note handler
    lowerRightLauncherUnreachable = 201,
    lowerLeftLauncherUnreachable = 202,
    indexerUnreachable = 210,
    intakeUnreachable = 211,

// intake
    intakeRotateUnreachable = 300,
    intakeRotateEncoderUnreachable = 301,
    intakeRotateCalibrationTimeout = 302,
    intakeRotateThermalLimit = 303,
    intakeLaserSensorFault = 304,

    driverTakeoverFailed = 1000,
    driverControllerUnreachable = 1001,
};