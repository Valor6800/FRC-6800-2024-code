#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"
#include <frc/DigitalInput.h>
#include "valkyrie/sensors/DebounceSensor.h"

class Feeder : public valor::BaseSubsystem
{
public:

    Feeder(frc::TimedRobot *robot, frc::DigitalInput* beamBreak);

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void InitSendable(wpi::SendableBuilder& builder);

    enum ROLLER_STATE
    {
        INTAKE,
        STAGNANT,
        OUTTAKE
    };

    struct x
    {
        ROLLER_STATE feederState;
        ROLLER_STATE intakeState;

        double intakeForwardSpeed;
        double intakeReverseSpeed;
        double feederForwardSpeed;
        double feederReverseSpeed;

    } state;

private:

    valor::NeoController feederMotor;
    valor::NeoController intakeMotor;
    frc::DigitalInput* beamBreak;
    valor::DebounceSensor debounceSensor;
};