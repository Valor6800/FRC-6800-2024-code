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
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Coast, true),
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

    debounceSensor.setGetter([this]() { return beamBreak->GetInWindow(); });
    intakeMotor.setMaxCurrent(60);
    intakeMotor.setVoltageCompensation(10);

    feederMotor.setVoltageCompensation(10);

    table->PutNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    table->PutNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    table->PutNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    table->PutNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);

    table->PutBoolean("Beam Trip", false);
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
        state.feederState = isBeamBreakTriggered() ? ROLLER_STATE::STAGNANT : ROLLER_STATE::INTAKE;
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
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);
}

void Feeder::assignOutputs()
{
    if(state.intakeState == ROLLER_STATE::INTAKE) {
        intakeMotor.setPower(state.intakeForwardSpeed);
    } else if(state.intakeState == ROLLER_STATE::OUTTAKE) {
        intakeMotor.setPower(state.intakeReverseSpeed);
    } else {
        intakeMotor.setPower(0);
    }

    if(state.feederState == ROLLER_STATE::INTAKE) {
        feederMotor.setPower(state.feederForwardSpeed);
    } else if(state.feederState == ROLLER_STATE::OUTTAKE) {
        feederMotor.setPower(state.feederReverseSpeed);
    } else {
        feederMotor.setPower(0);
    }
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
        "debounceSensor",
        [this] {return debounceSensor.getSensor();},
        nullptr
    );

    builder.AddBooleanProperty(
        "Banner Raw",
        [this]{return isBeamBreakTriggered();},
        nullptr
    );
}
