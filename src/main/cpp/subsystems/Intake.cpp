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
    RollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Brake, false),
    // ActivationMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Brake, false),
    beam(_beamBreak),
    debounce(_robot, "Intake")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{
}

void Intake::resetState()
{
    state.activation = STOWED;
    state.intake = STAGNANT;
    state.detection = NOTE_NOTDETECTED;
}

void Intake::init()
{
    RollerMotor.setConversion(1.0 / OTB_ROLLER_GEAR_RATIO * 360);
    // ActivationMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutNumber("Intake Power", ROLLER_MOTOR_INTAKE_POWER);
    table->PutNumber("Outtake Power", ROLLER_MOTOR_OUTTAKE_POWER);
    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assessInputs()
{
    if(operatorGamepad->GetRightTriggerAxis() != 0.0)
    {
        state.intake = INTAKING;
        state.activation = DEPOLOYED;
    }
    else
    {
        if(operatorGamepad->GetLeftTriggerAxis() != 0.0)
        {
            state.intake = OUTTAKE;
            state.activation = DEPOLOYED;
        }
        else
        {
            state.intake = STAGNANT;
            state.activation = STOWED;
        }
    }
    if(beam->Get())
    {
        state.detection = NOTE_DETECTED;
    }
    else
    {
        state.detection = NOTE_NOTDETECTED;
    }
}

void Intake::analyzeDashboard()
{
    IntakeRotMaxSpeed = table->GetNumber("Intake Power", ROLLER_MOTOR_INTAKE_POWER);
    OuttakeRotMaxSpeed = table->GetNumber("Outtake Power", ROLLER_MOTOR_OUTTAKE_POWER);

    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assignOutputs()
{
    if(state.activation == DEPOLOYED)
    {
        // ActivationMotor.setPosition(OTB_DEPLOYED_POSITION);
    }
    else if(state.activation == STOWED)
    {
        // ActivationMotor.setPosition(OTB_STOWED_POSITION);
    }

    if(state.intake == INTAKING)
    {
        RollerMotor.setPower(IntakeRotMaxSpeed);
    }
    else if(state.intake == OUTTAKE)
    {
        RollerMotor.setPower(OuttakeRotMaxSpeed);
    }
    else
    {
        RollerMotor.setPower(0);
    }
    if(state.detection = NOTE_DETECTED)
    {
        // LED ON
    }
    else
    {
        // LED OFF
    }
}

double Intake::getOTBRollerSpeed()
{
    return RollerMotor.getSpeed();
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "isDropdown",
        [this] {return state.activation;},
        nullptr
    );

    builder.AddDoubleProperty(
        "isIntaking",
        [this] {return state.intake;},
        nullptr
    );

    builder.AddDoubleProperty(
        "isNote",
        [this] {return state.detection;},
        nullptr
    );

    builder.AddDoubleProperty(
        "OTBrollerSpeed",
        [this] {return getOTBRollerSpeed();},
        nullptr
    );

    /*builder.AddDoubleProperty(
        "debounceTest",
        [this] {return debounce.}
    ) */

    builder.AddBooleanProperty(
        "beamDetection",
        [this] {return beam->Get();},
        nullptr
    );
}