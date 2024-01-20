# pragma once 

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
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalOutput.h>

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"

class Shooter : public valor::BaseSubsystem
{
public:
    //valor::NeoController pivotMotors;
    valor::NeoController LeftflywheelMotors;
    valor::NeoController RightflywheelMotors;

    Shooter(frc::TimedRobot *robot, frc::DigitalInput* beamBreak);

    ~Shooter();

    void resetState();

    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    
    void InitSendable(wpi::SendableBuilder& builder);

    enum FlywheelState
    {
        NOT_SHOOTING,
        SPOOLED,
        SHOOTING
    };

    enum PivotState
    {
        SUBWOOFER,
        PODIUM,
        STARTING_LINE,
        TRACKING
    };

    struct x
    {
        PivotState pivot;
        FlywheelState flywheel;
    } state;

private:
    units::degree_t calculatingPivotingAngle;

    valor::PIDF pivotPID;

    frc::DigitalInput* beamBreak;

    double leftShootPwr, rightShootPwr, leftSpooledPwr, rightSpooledPwr;
};