#pragma once

#include "frc/AnalogTriggerOutput.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"
#include <frc/DigitalInput.h>
#include "valkyrie/sensors/DebounceSensor.h"

#include "valkyrie/sensors/CurrentSensor.h"
#include "frc/PWM.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include<networktables/NetworkTableInstance.h>


class Feeder : public valor::BaseSubsystem
{
public:

    Feeder(frc::TimedRobot *robot, frc::AnalogTrigger* beamBreak);

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    bool isIntake();
    bool isBeamBreakTriggered();

    void InitSendable(wpi::SendableBuilder& builder);

    enum ROLLER_STATE
    {
        SHOOT,
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
        
        bool beamTrip;

    } state;

private:
    valor::NeoController feederMotor;
    valor::NeoController intakeMotor;
    frc::AnalogTrigger* beamBreak;
    valor::DebounceSensor debounceSensor;
    valor::CurrentSensor currentSensor;

    bool BeamBreakTriggered;
    bool intakeTest;

};
