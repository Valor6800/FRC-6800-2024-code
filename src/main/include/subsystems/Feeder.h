#pragma once

#include "frc/AnalogTriggerOutput.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"
#include <frc/DigitalInput.h>
#include <frc/PWM.h>

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

    void InitSendable(wpi::SendableBuilder& builder);

    enum ROLLER_STATE
    {
        SHOOT,
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
        
        bool beamTrip;

    } state;

private:
    units::microsecond_t LED_OFF = 1815.0_us;
    units::microsecond_t LED_ON = 1865.0_us;
    frc::PWM blinkin{DIOPorts::BLINKIN};

    bool isBeamBreakTriggered();

    valor::NeoController feederMotor;
    valor::NeoController intakeMotor;
    frc::AnalogTrigger* beamBreak;

    bool BeamBreakTriggered;
    bool IntakeTest;
};
