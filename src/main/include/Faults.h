#pragma once

enum class FaultIdentifier {

    allianceUnavailable = 400,

// intake
    intakeRotateUnreachable = 300,
    intakeRotateEncoderUnreachable = 301,
    intakeRotateCalibrationTimeout = 302,
    intakeRotateThermalLimit = 303,
    intakeLaserSensorFault = 304,

    drivebaseTakeoverFailed = 1000,
    controllerUnreachable = 1001,
    incorrectController = 1002,

// lift
    liftIntakeCollisionLock = 200,
    liftUnreachable = 201,
    shoulderUnreachable = 202,
    twistUnreachable = 203,
    gripperUnreachable = 204,

    driveMotorUnreachable = 100,
    steerMotorUnreachable = 101,
    steerEncoderUnreachable = 102,
};