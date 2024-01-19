# pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

class Climber : public valor::BaseSubsystem
{
public:
    valor::NeoController RightClimbMotor;
    valor::NeoController LeftClimbMotor;

    Climber(frc::TimedRobot *robot);

    ~Climber();

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void InitSendable(wpi::SendableBuilder& builder);

    void extend();
    void climb();

    double getClimberSpeed();

    struct x
    {
        bool isInClimb;
        bool isExtended;
    } state;

private:

    double climbMaxSpeed;

    valor::PIDF climberPID;
};