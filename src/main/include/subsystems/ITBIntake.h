# pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

class ITBIntake : public valor::BaseSubsystem
{
public:
    valor::NeoController ITBIntakeRollerMotor;

    ITBIntake(frc::TimedRobot *robot);

    ~ITBIntake();

    void init();

    // void assessInputs();
    // void analyzeDashboard();
    // void assignOutputs();

    // void resetState();

    // void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        
        bool ITBisIntaking; 

    } state;

private:

    double ITBintakeRotMaxSpeed;

};