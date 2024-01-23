#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/DebounceSensor.h"

#include "frc/DigitalInput.h"

class Intake : public valor::BaseSubsystem
{
public:

    Intake(frc::TimedRobot *robot, frc::DigitalInput *_beamBreak);

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void InitSendable(wpi::SendableBuilder& builder);

    enum INTAKE_STATE
    {
        INTAKE,
        STAGNANT,
        OUTTAKE
    };

    struct x
    {
        INTAKE_STATE intakeState;
        bool activationState;
        bool detectionState;

        double intakeForwardSpeed;
        double intakeReverseSpeed;

    } state;

private:

    valor::NeoController rollerMotor;
    // valor::NeoController ActivationMotor;

    frc::DigitalInput* beam;
    valor::DebounceSensor debounce;
};