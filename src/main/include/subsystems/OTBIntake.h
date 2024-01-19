#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

class OTBIntake : public valor::BaseSubsystem
{
public:
    valor::NeoController OTBIntakeRollerMotor;
    valor::NeoController OTBDropDownMotor;

    OTBIntake(frc::TimedRobot *robot);

    ~OTBIntake();

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void dropDown();
    void rollerIntake();

    double getOTBRollerSpeed();

    void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        bool OTBisIntaking;
        bool dropDown;
    } state;

private:

    double OTBintakeRotMaxSpeed;
    double dropDownMaxSpeed;
};