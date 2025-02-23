#pragma once

enum class FaultIdentifier {
// note handler
    lowerRightLauncherUnreachable = 201,
    lowerLeftLauncherUnreachable = 202,
    indexerUnreachable = 210,
    intakeUnreachable = 211,
    driverTakeoverFailed = 300,
    driverControllerUnreachable = 301,
};