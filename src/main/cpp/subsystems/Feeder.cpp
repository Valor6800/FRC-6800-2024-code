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
#define FEEDER_INTAKE_POWER 0.3f
#define FEEDER_REVERSE_POWER -0.5f

Feeder::Feeder(frc::TimedRobot *_robot, frc::AnalogTrigger* _beamBreak) :
    valor::BaseSubsystem(_robot, "Feeder"),
    intakeMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Brake, true),
    beamBreak(_beamBreak),
    debounceSensor(_robot, "FeederBanner")
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

    debounceSensor.setGetter([this] { return !beamBreak->GetInWindow(); });
    debounceSensor.setRisingEdgeCallback([this] {
        state.beamTrip = true;
        feederMotor.setPower(0);
    });

    table->PutNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    table->PutNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    table->PutNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    table->PutNumber("Feeder Intake Power", FEEDER_INTAKE_POWER);
    table->PutNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    table->PutBoolean("Beam Trip", false);
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
    
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    state.feederForwardSpeed = table->GetNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    state.feederIntakeSpeed = table->GetNumber("Feeder Intake Power", FEEDER_INTAKE_POWER);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    if (state.feederState == ROLLER_STATE::SHOOT) {
        state.beamTrip = false;
    }
    blinkin.SetPulseTime(state.beamTrip ? LED_ON : LED_OFF);
}

void Feeder::assignOutputs()
{
    if(state.intakeState == ROLLER_STATE::SHOOT) {
        intakeMotor.setPower(state.intakeForwardSpeed);
    } else if(state.intakeState == ROLLER_STATE::INTAKE) {
        intakeMotor.setPower(state.beamTrip ? 0 : state.intakeForwardSpeed);
    } else if(state.intakeState == ROLLER_STATE::OUTTAKE) {
        intakeMotor.setPower(state.intakeReverseSpeed);
    } else {
        intakeMotor.setPower(0);
    }
    
    if (state.feederState == ROLLER_STATE::SHOOT) {
        feederMotor.setPower(state.feederForwardSpeed);
    } else if(state.feederState == ROLLER_STATE::INTAKE) {
        feederMotor.setPower(state.beamTrip ? 0 : state.feederIntakeSpeed);
    } else if(state.feederState == ROLLER_STATE::OUTTAKE) {
        feederMotor.setPower(state.feederReverseSpeed);
    } else {
        feederMotor.setPower(0);
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
        "Beam Trip",
        [this]{return state.beamTrip;},
        nullptr
    );
}
