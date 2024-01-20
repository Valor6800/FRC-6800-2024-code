#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

class Intake : public valor::BaseSubsystem
{
public:
    valor::NeoController IntakeRollerMotor;
    valor::NeoController OTBDropDownMotor;

    Intake(frc::TimedRobot *robot);

    ~Intake();

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
        enum Activation_State
        {
            DEPOLOYED,
            STOWED
        };

        enum Intake_State
        {
            INTAKING,
            STAGNANT,
            OUTTAKE
        };

        enum Detection_State
        {
            NOTE_DETECTED,
            NOTE_NOTDETECTED
        };
        int activation;
        int intake;
        int detection;

    } state;

private:

    double IntakeRotMaxSpeed;
    double dropDownMaxSpeed;
};