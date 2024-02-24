#include "subsystems/Feeder.h"
#include <iostream>
#include <math.h>
#include <frc/AnalogOutput.h>
#include <frc/AnalogTrigger.h>
#include "frc/AnalogTriggerType.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"
#include "frc/AnalogTriggerOutput.h"

#define INTAKE_FORWARD_POWER 1.0f
#define INTAKE_REVERSE_POWER -1.0f

#define FEEDER_FORWARD_POWER 0.5f
#define FEEDER_REVERSE_POWER -0.5f

Feeder::Feeder(frc::TimedRobot *_robot, frc::AnalogTrigger* _beamBreak) :
    valor::BaseSubsystem(_robot, "Feeder"),
    intakeMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Brake, true),
    currentSensor(_robot, subsystemName),
    beamBreak(_beamBreak),
    debounceSensor(_robot, "Feeder")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Feeder::resetState()
{
    blinkin.SetPulseTime(LED_OFF);
    state.intakeState = STAGNANT;
    state.feederState = STAGNANT;
}

void Feeder::init()
{
    resetState();

    intakeMotor.setMaxCurrent(60);
    intakeMotor.setVoltageCompensation(10);

    feederMotor.setVoltageCompensation(10);

    table->PutNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    table->PutNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    table->PutNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    table->PutNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    currentSensor.setGetter([this]() {return intakeMotor.getCurrent(); });
    currentSensor.setGetter([this]() {return feederMotor.getCurrent(); });

    currentSensor.setSpikeCallback([this]() {return feederMotor.getCurrent(); });



    table->PutBoolean("Beam Trip", false);
    table->PutBoolean("IntakeTest", false);

    IntakeTest = false;



}

void Feeder::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;

    if (driverGamepad->rightTriggerActive()) {
        state.intakeState = ROLLER_STATE::SHOOT;
        state.feederState = ROLLER_STATE::SHOOT;
    } else if (driverGamepad->GetRightBumper()) {
        state.intakeState = ROLLER_STATE::INTAKE;
        state.feederState = ROLLER_STATE::INTAKE;
    } else if(operatorGamepad->GetLeftBumper()) {
        state.intakeState = ROLLER_STATE::OUTTAKE;
        state.feederState = ROLLER_STATE::OUTTAKE;
    } else {
        state.intakeState = ROLLER_STATE::STAGNANT;
        state.feederState = ROLLER_STATE::STAGNANT;
    }

}

void Feeder::analyzeDashboard()
{
    blinkin.SetPulseTime(isBeamBreakTriggered() ? LED_ON : LED_OFF);
    
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    state.feederForwardSpeed = table->GetNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    if (state.feederState == ROLLER_STATE::SHOOT) {
        state.beamTrip = false;
    } else {
        state.beamTrip |= isBeamBreakTriggered();
    }
}

void Feeder::assignOutputs()
{
    if(state.intakeState == ROLLER_STATE::INTAKE || state.intakeState == ROLLER_STATE::SHOOT) {
        intakeMotor.setPower(state.intakeForwardSpeed);
    } else if(state.intakeState == ROLLER_STATE::OUTTAKE) {
        IntakeTest = false;

        intakeMotor.setPower(state.intakeReverseSpeed);
    } else {
        IntakeTest = false;

        intakeMotor.setPower(0);
    }
    
    if (state.feederState == ROLLER_STATE::SHOOT) {
        feederMotor.setPower(state.feederForwardSpeed);
    } else if(state.feederState == ROLLER_STATE::INTAKE) {
        feederMotor.setPower(state.beamTrip ? 0 : state.feederForwardSpeed);
    } else if(state.feederState == ROLLER_STATE::OUTTAKE) {
        IntakeTest = false;
        IntakeTest = false;
        feederMotor.setPower(state.feederReverseSpeed);
    } else {
        IntakeTest = false;

        feederMotor.setPower(0);
    }
    
    table->PutBoolean("IntakeTest", IntakeTest);
    table->PutBoolean("Beam Trip", isBeamBreakTriggered());

}

bool Feeder::isBeamBreakTriggered()
{
    return !beamBreak->GetInWindow();
}

void Feeder::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "intakeState",
        [this] {return state.intakeState;},
        nullptr
    );

    builder.AddDoubleProperty(
        "feederState",
        [this] {return state.feederState;},
        nullptr
    );

    builder.AddBooleanProperty(
        "Banner Raw",
        [this]{return isBeamBreakTriggered();},
        nullptr
    );
}
