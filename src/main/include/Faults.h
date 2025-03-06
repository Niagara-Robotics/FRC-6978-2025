#pragma once

enum class FaultIdentifier {
// note handler

// intake
    intakeRotateUnreachable = 300,
    intakeRotateEncoderUnreachable = 301,
    intakeRotateCalibrationTimeout = 302,
    intakeRotateThermalLimit = 303,
    intakeLaserSensorFault = 304,

    driverTakeoverFailed = 1000,
    driverControllerUnreachable = 1001,
    operatorControllerUnreachable = 1101,

// lift
    liftIntakeCollisionLock = 100
};