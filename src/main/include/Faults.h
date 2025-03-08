#pragma once

enum class FaultIdentifier {
// note handler

// intake
    intakeRotateUnreachable = 300,
    intakeRotateEncoderUnreachable = 301,
    intakeRotateCalibrationTimeout = 302,
    intakeRotateThermalLimit = 303,
    intakeLaserSensorFault = 304,

    drivebaseTakeoverFailed = 1000,
    controllerUnreachable = 1001,

// lift
    liftIntakeCollisionLock = 100,

    driveMotorUnreachable = 400,
    steerMotorUnreachable = 401,
    steerEncoderUnreachable = 402,
};