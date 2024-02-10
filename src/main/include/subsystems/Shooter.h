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

    void getTargetPivotAngle(bool laser);
    void getArcTargetPivotAngle();
    /**
     * @brief Returns the projectile speed in the x-direction or the y-direction
     * @param type A boolean to represent direction, X is true, Y is false
    */
    units::velocity::meters_per_second_t getProjectileSpeed(bool type);
    void calculateShootingMovingAngle();
    units::radian_t getPivotErrorAngle();
    double solveCubic(double a, double b, double c, double d);
    double solveQuartic(double a, double b, double c, double d, double e);
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
    } state;

private:

    Drivetrain *drivetrain;
    valor::PIDF pivotPID;
    frc::DigitalInput* beamBreak;
};
