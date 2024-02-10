# pragma once 

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/NeoController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"
#include "Drivetrain.h"

class Shooter : public valor::BaseSubsystem
{
public:
    valor::NeoController pivotMotor;
    valor::NeoController leftFlywheelMotor;
    valor::NeoController rightFlywheelMotor;

    Shooter(frc::TimedRobot *robot, frc::DigitalInput* beamBreak, Drivetrain *drivetrain);

    void resetState();

    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    /**
     * @brief Sets the target angle of the pivot when the robot is still
     * @param laser If true, disregards gravity, if false, takes gravity into account
    */
    void getTargetPivotAngle(bool laser);
    /**
     * @brief Sets the target angle of the pivot when the robot is still and takes
     * gravity into account
    */
    void getArcTargetPivotAngle();
    /**
     * @brief Returns the projectile speed in the x-direction or the y-direction
     * @param type A boolean to represent direction, X is true, Y is false
    */
    units::velocity::meters_per_second_t getProjectileSpeed(bool type);
    /**
     * @brief Sets the target angle of the pivot when the robot is moving
    */
    void calculateShootingMovingAngle();
    /**
     * @brief Returns the error angle between the target pivot angle and the current pivot angle
    */
    units::radian_t getPivotErrorAngle();
    /**
     * @brief Defines a cubic function with coefficients and returns the value of f(x)
     * @param a First coefficient of the function
     * @param b Second coefficient of the function
     * @param c Third coefficient of the function
     * @param d Constant in the function
     * @param x The variable used in the funciton, the input of the function
    */
    double cubicFunction(double a, double b, double c, double d, double x);
    /**
     * @brief Defines the derivative of a cubic function
     * @param a The first coefficient of the cubic
     * @param b The second coefficient of the cubic
     * @param c The third coeifficient of the cubic
     * @param x The input of the function
    */
    double deriveCubic(double a, double b, double c, double x);
    /**
     * @brief Defines a quartic function with coefficients and returns the value of f(x)
     * @param a First coefficient of the function
     * @param b Second coefficient of the function
     * @param c Third coefficient of the function
     * @param d Fourth coefficient in the function
     * @param e Constant of the funtion
     * @param x The variable used in the function, the input of the function
    */
    double quarticFunction(double a, double b, double c, double d, double e, double x);
    double deriveQuartic(double a, double b, double c, double d, double x);
    /**
     * @brief Returns the roots of a polynomial in form ax^2 + bx +c
     * @param a The first coefficient in a quadratic
     * @param b The second coefficient in a quadratic
     * @param c The third coefficient in a quadratic
    */
    std::pair<double, double> solveQuadratic(double a, double b, double c);
    std::pair<double, double> solveCubic(double a, double b, double c, double d);
    double solveCubicNewtonMethod(double a, double b, double c, double d, double start);
    double solveQuartic(double a, double b, double c, double d, double e);
    double solveQuarticNewtonMethod(double a, double b, double c, double d, double e, double start);
    double calculateRootsT(double accX, double accY, double velX, double velY, double posX, double posY);
    
    void InitSendable(wpi::SendableBuilder& builder);

    enum FLYWHEEL_STATE
    {
        NOT_SHOOTING,
        SPOOLED,
        SHOOTING
    };

    enum PIVOT_STATE
    {
        SUBWOOFER,
        PODIUM,
        STARTING_LINE,
        TRACKING
    };

    struct x
    {
        PIVOT_STATE pivot;
        FLYWHEEL_STATE flywheelState;

        units::angular_velocity::revolutions_per_minute_t leftFlywheelTargetVelocity;
        units::angular_velocity::revolutions_per_minute_t rightFlywheelTargetVelocity;
        units::degree_t pivotAngle;
        units::radian_t targetPivotAngle;
        PIVOT_STATE pivotState;
    } state;

private:

    Drivetrain *drivetrain;
    valor::PIDF flywheelPID;
    valor::PIDF pivotPID;
    frc::DigitalInput* beamBreak;
};
