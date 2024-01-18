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

    void init();

    // void assessInputs();
    // void analyzeDashboard();
    // void assignOutputs();

    // void resetState();

    // void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        bool isInClimb;
    } state;

private:

    double climbMaxSpeed;

    valor::PIDF climberPID;
};