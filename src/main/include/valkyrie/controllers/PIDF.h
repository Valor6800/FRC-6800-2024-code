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
    double P = 0.1;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Feedforward term
    double F = 0.000244;
    /// Max velocity: revolutions per 1s
    double velocity = 1500;
    /// Max acceleration: revolutions per 1s^2
    double acceleration = 15000;
    /// Max jerk: revolutions per 1s^3
    double jerk = 0;
    /// Minimum error threshold
    double error = 0.5;

    double aFF = 0;
    double aFFTarget = 90;
    FeedForwardType aFFType = FeedForwardType::LINEAR;
};
}
