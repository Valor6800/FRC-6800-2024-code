#include "subsystems/Feeder.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define INTAKE_FORWARD_POWER 0.8f
#define INTAKE_REVERSE_POWER -0.8f

#define FEEDER_FORWARD_POWER 0.8f
#define FEEDER_REVERSE_POWER -0.8f

Feeder::Feeder(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Feeder"),
    intakeMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Coast, true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Shoot sequence-feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::INTAKE;
                }
            ),
            frc2::WaitCommand(1_s),
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = false;
                    state.feederState = Feeder::ROLLER_STATE::STAGNANT;
                }
            )
        )
    ).ToPtr());
}

void Feeder::resetState()
{
    state.intakeState = STAGNANT;
    state.feederState = STAGNANT;
}

void Feeder::init()
{
    resetState();

    table->PutNumber("Intake Forward Power", INTAKE_FORWARD_POWER);
    table->PutNumber("Intake Reverse Power", INTAKE_REVERSE_POWER);

    table->PutNumber("Feeder Forward Power", FEEDER_FORWARD_POWER);
    table->PutNumber("Feeder Reverse Power", FEEDER_REVERSE_POWER);
}

void Feeder::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr )
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
}