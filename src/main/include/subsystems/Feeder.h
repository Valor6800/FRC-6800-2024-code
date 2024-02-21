#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/sensors/CurrentSensor.h"
#include <frc/DigitalInput.h>
#include "valkyrie/sensors/DebounceSensor.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include<networktables/NetworkTableInstance.h>

#include "frc/DigitalInput.h"
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
        OUTTAKE,
        SPIKED
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

    valor::NeoController intakeMotor;
    valor::NeoController feederMotor;

    valor::CurrentSensor currentSensor;
    frc::DigitalInput* beamBreak;
    valor::DebounceSensor debounceSensor;

    bool IntakeTest;
};