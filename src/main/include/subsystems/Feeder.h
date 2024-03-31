#pragma once

#include "frc/AnalogTriggerOutput.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"
#include <frc/DigitalInput.h>
#include <frc/PWM.h>
#include "valkyrie/sensors/DebounceSensor.h"
#include "valkyrie/sensors/CANdleSensor.h"

class Feeder : public valor::BaseSubsystem
{
public:

    Feeder(frc::TimedRobot *, frc::AnalogTrigger*, frc::AnalogTrigger*, valor::CANdleSensor*);

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
        OUTTAKE,
        TUNING
    };

    struct x
    {
        ROLLER_STATE feederState;
        ROLLER_STATE intakeState;
        bool stageTrip;
        bool feedTrip;
        double tuningPower;
    } state;

private:   
    valor::NeoController intakeMotor;
    valor::NeoController intakeBackMotor;
    valor::NeoController feederMotor;
    frc::AnalogTrigger* feederBeamBreak;
    valor::DebounceSensor feederDebounceSensor;
    frc::AnalogTrigger* stageBeamBreak;    
    valor::DebounceSensor stageDebounceSensor;

    valor::CANdleSensor *leds;
};
