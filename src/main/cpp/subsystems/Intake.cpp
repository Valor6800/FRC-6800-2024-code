#include "subsystems/Intake.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"
#include "valkyrie/sensors/DebounceSensor.h"

#define OTB_ROLLER_GEAR_RATIO 4.0f
#define OTB_DROPDOWN_GEAR_RATIO 3.0f

#define OTB_DEPLOYED_POSITION 33.0f
#define OTB_STOWED_POSITION 0.0f

#define ROLLER_MOTOR_INTAKE_POWER 0.8f
#define ROLLER_MOTOR_OUTTAKE_POWER -0.8f

Intake::Intake(frc::TimedRobot *_robot, frc::DigitalInput *_beamBreak) :
    valor::BaseSubsystem(_robot, "Intake"),
    rollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    // ActivationMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Coast, false),
    beam(_beamBreak),
    debounce(_robot, "Intake")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Intake::resetState()
{
    state.activationState = false;
    state.detectionState = false;
    state.intakeState = STAGNANT;
}

void Intake::init()
{
    rollerMotor.setConversion(1.0 / OTB_ROLLER_GEAR_RATIO * 360);
    // ActivationMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutNumber("Intake Power", ROLLER_MOTOR_INTAKE_POWER);
    table->PutNumber("Outtake Power", ROLLER_MOTOR_OUTTAKE_POWER);
}

void Intake::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr )
        return;

    if (driverGamepad->GetLeftBumper()) {
        state.intakeState = INTAKE_STATE::INTAKE;
        state.activationState = true;
    } else if (operatorGamepad->DPadDown()) {
        state.intakeState = INTAKE_STATE::OUTTAKE;
        state.activationState = true;
    } else {
        state.intakeState = INTAKE_STATE::STAGNANT;
        state.activationState = false;
    }
    state.detectionState = !beam->Get(); 
}

void Intake::analyzeDashboard()
{
    state.intakeForwardSpeed = table->GetNumber("Intake Power", ROLLER_MOTOR_INTAKE_POWER);
    state.intakeReverseSpeed = table->GetNumber("Outtake Power", ROLLER_MOTOR_OUTTAKE_POWER);
}

void Intake::assignOutputs()
{
    // ActivationMotor.setPosition(state.activation ? OTB_DEPLOYED_POSITION : OTB_STOWED_POSITION);

    if (state.intakeState == INTAKE_STATE::INTAKE) {
        rollerMotor.setPower(state.intakeForwardSpeed);
    } else if(state.intakeState == INTAKE_STATE::OUTTAKE) {
        rollerMotor.setPower(state.intakeReverseSpeed);
    } else {
        rollerMotor.setPower(0);
    }
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "isDropdown",
        [this] {return state.activationState;},
        nullptr
    );

    builder.AddDoubleProperty(
        "isIntaking",
        [this] {return state.intakeState;},
        nullptr
    );

    builder.AddDoubleProperty(
        "isNote",
        [this] {return state.detectionState;},
        nullptr
    );

    /*builder.AddDoubleProperty(
        "debounceTest",
        [this] {return debounce.}
    ) */

    builder.AddBooleanProperty(
        "beamDetection",
        [this] {return !beam->Get();},
        nullptr
    );
}