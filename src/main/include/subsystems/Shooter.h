# pragma once 

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalOutput.h>
#include "valkyrie/sensors/DebounceSensor.h"

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"

class Shooter : public valor::BaseSubsystem
{
public:

    Shooter(frc::TimedRobot *robot, Drivetrain *drivetrain, frc::AnalogTrigger*, frc::AnalogTrigger*);

    void resetState();

    void init();
    bool getSpooledState();
    bool getTrackingState();
    bool getPitModeState();


    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void calculatePivotAngle();
    
    void InitSendable(wpi::SendableBuilder& builder);

    enum FLYWHEEL_STATE
    {
        NOT_SHOOTING,
        SHOOTING,
    };

    enum PIVOT_STATE
    {
        DISABLED,
        SUBWOOFER,
        PODIUM,
        WING,
        TRACKING,
        AMP,
        ORBIT,
        TUNING,
        LOAD,
        AUTO_FAR_LOW,
        AUTO_FAR_HIGH,
        AUTO_NEAR,
        FORCE_INTAKE,
        AUTO_SUBWOOFER,
    };

    struct x
    {
        double tuningSetpoint;
        double tuningSpeed;
        double pivotOffset;
        double tuningOffset;

        PIVOT_STATE pivotState;
        FLYWHEEL_STATE flywheelState;

        units::degree_t pivotAngle;
        units::degree_t calculatingPivotingAngle;

        bool ignoreLoad;
        bool otherSide;
        bool insideWing;

        bool reverseFlywheels;
    } state;

private:
    Drivetrain *drivetrain;
    valor::PhoenixController* pivotMotors;
    frc::AnalogTrigger* feederBeamBreak;
    frc::AnalogTrigger* feederBeamBreak2;

    valor::NeoController leftFlywheelMotor;
    valor::NeoController rightFlywheelMotor;
    bool trackingTest;

};
