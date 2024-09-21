#include "subsystems/Feeder.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#define INTAKE_FORWARD_POWER 0.8f
#define INTAKE_REVERSE_POWER -0.8f

#define FEEDER_FORWARD_POWER 0.8f
#define FEEDER_REVERSE_POWER -0.8f

Feeder::Feeder(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Feeder"),
    intakeMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Coast, true),

    currentSensor(_robot, subsystemName),

    beamBreak(_beamBreak),
    debounceSensor(_robot, "Feeder")

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}


void Feeder::resetState()
{
    state.intakeState = STAGNANT;
    state.feederState = STAGNANT;
}

void Feeder::init()
{
    resetState();

    debounceSensor.setGetter([this]() { return beamBreak->Get(); });

    table->PutNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    table->PutNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    table->PutNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    table->PutNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    table->PutBoolean("IntakeTest", false);

    currentSensor.setGetter([this]() {return intakeMotor.getCurrent(); });
    currentSensor.setGetter([this]() {return feederMotor.getCurrent(); });

    currentSensor.setSpikeCallback([this]() {return feederMotor.getCurrent(); });

}

void Feeder::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;

    if (driverGamepad->rightTriggerActive()) {
        state.intakeState = ROLLER_STATE::INTAKE;
        state.feederState = ROLLER_STATE::INTAKE;
    } else if (driverGamepad->GetRightBumper() || driverGamepad->GetLeftBumper()) {
        state.intakeState = ROLLER_STATE::INTAKE;
        state.feederState = ROLLER_STATE::STAGNANT;
    } else if(operatorGamepad->DPadDown()) {
        state.intakeState = ROLLER_STATE::OUTTAKE;
        state.feederState = ROLLER_STATE::OUTTAKE;
    } else {
        state.intakeState = ROLLER_STATE::STAGNANT;
        state.feederState = ROLLER_STATE::STAGNANT;
    }
}

void Feeder::analyzeDashboard()
{
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    state.feederForwardSpeed = table->GetNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);
}

void Feeder::assignOutputs()
{
    if(state.intakeState == ROLLER_STATE::INTAKE) {
        IntakeTest = true;
        intakeMotor.setPower(state.intakeForwardSpeed);
    } else if(state.intakeState == ROLLER_STATE::OUTTAKE) {
        IntakeTest = false;
        intakeMotor.setPower(state.intakeReverseSpeed);
    } else {
        IntakeTest = false;
        intakeMotor.setPower(0);
    }

    if(state.feederState == ROLLER_STATE::INTAKE) {
        IntakeTest = true;
        feederMotor.setPower(state.feederForwardSpeed);
    } else if(state.feederState == ROLLER_STATE::OUTTAKE) {
        IntakeTest = false;
        feederMotor.setPower(state.feederReverseSpeed);
    } else {
        IntakeTest = false;
        feederMotor.setPower(0);
    }

    table->PutBoolean("IntakeTest", IntakeTest);

    if(state.intakeState==ROLLER_STATE::SPIKED){
        table->PutBoolean("SpikeTest", true);
    }

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
        "debounceSensor",
        [this] {return debounceSensor.getSensor();},
        nullptr
    );
}
