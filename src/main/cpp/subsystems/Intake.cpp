#include "subsystems/Intake.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#define OTB_ROLLER_GEAR_RATIO 4.0f
#define OTB_DROPDOWN_GEAR_RATIO 3.0f

#define OTB_DEPLOYED_POSITION 33.0f
#define OTB_STOWED_POSITION 0.0f

Intake::Intake(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Intake"),
    RollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Brake, false),
    ActivationMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Brake, false),
    beam(DIOPorts::BEAM_DIO_PORT)
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
    ActivationMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assessInputs()
{
    if(operatorGamepad->GetXButtonPressed())
    {
        state.intake = INTAKING;
        state.activation = DEPOLOYED;
    }
    else
    {
        if(operatorGamepad->GetYButtonPressed())
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
    if(beam.Get())
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
    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assignOutputs()
{
    if(state.activation == DEPOLOYED)
    {
        ActivationMotor.setPosition(OTB_DEPLOYED_POSITION);
    }
    else if(state.activation == STOWED)
    {
        ActivationMotor.setPosition(OTB_STOWED_POSITION);
    }

    if(state.intake == INTAKING)
    {
        RollerMotor.setPower(1);
    }
    else if(state.intake == OUTTAKE)
    {
        RollerMotor.setPower(-1);
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

    builder.AddBooleanProperty(
        "beamDetection",
        [this] {return beam.Get();},
        nullptr
    );
}