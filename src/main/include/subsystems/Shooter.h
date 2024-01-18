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

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"


class Shooter : public valor::BaseSubsystem
{
public:
    valor::NeoController ShooterAngleControlMotor;
    valor::NeoController RightWheelsShootingMotor;
    valor::NeoController LeftWheelShootingMotor;

    valor::Gamepad *operatorGamepad;
    valor::Gamepad *driverGamepad;

    Shooter(frc::TimedRobot *robot);

    ~Shooter();

    void resetState();

    void setGamepads(valor::Gamepad *_operatorGamepad, valor::Gamepad *_driverGamepad)
    {
        operatorGamepad = _operatorGamepad;
        driverGamepad = _driverGamepad;
    }

    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    // void resetState();

    // void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        bool rightSideShooting;
        bool leftSideShooting;
        bool isShooting;
        double shootingSpeed;
        double angle;
    } state;

private:

    double shootingMaxSpeed;
    double angleAdjustMaxSpeed;

    valor::PIDF shooterAngle;
};