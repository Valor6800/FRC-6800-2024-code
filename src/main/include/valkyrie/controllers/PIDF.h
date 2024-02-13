#pragma once

namespace valor {

enum FeedForwardType
{
    LINEAR,
    CIRCULAR
};

/**
 * @brief Container to hold PID and feed forward values for the motor controller
 */
struct PIDF
{
    /// Proportion control of the feedback term
    double P = 0.0;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Max velocity: revolutions per 1s
    double maxVelocity = 0.0;
    /// Max acceleration: revolutions per 1s^2
    double maxAcceleration = 0.0;
    /// Max jerk: revolutions per 1s^3
    double maxJerk = 0.0;
    /// Minimum error threshold
    double error = 0.0;

    double aFF = 0;
    double aFFTarget = 90;
    FeedForwardType aFFType = FeedForwardType::LINEAR;
};
}
