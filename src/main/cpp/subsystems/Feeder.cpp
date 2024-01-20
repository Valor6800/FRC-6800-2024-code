#include "subsystems/Indexer.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define INDEXER_GEAR_RATIO 0.0f
#define ITB_ROLLER_GEAR_RATIO 2.0f

Indexer::Indexer(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Indexer"),
    NoteHandoffMotor(CANIDs::HANDOFF_CONTROLLER, valor::NeutralMode::Brake, false),
    ITBRollerMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Indexer::~Indexer()
{   
}

void Indexer::resetState()
{
    state.inHandoff = false;
    state.isIndexIntake = false;
}

void Indexer::init()
{
    NoteHandoffMotor.setConversion(1.0 / INDEXER_GEAR_RATIO * 360);
    ITBRollerMotor.setConversion(1.0 / ITB_ROLLER_GEAR_RATIO * 360);

    resetState();

    table->PutBoolean("In Handoff?", state.inHandoff);
    table->PutBoolean("Roller Intaking?", state.isIndexIntake);
}

void Indexer::assessInputs()
{
    if(operatorGamepad->GetAButtonPressed())
    {
        state.inHandoff = true;
    }
    if(operatorGamepad->GetBButtonPressed())
    {
        state.isIndexIntake = true;
    }
}

void Indexer::analyzeDashboard()
{
    table->PutBoolean("In handoff?", state.inHandoff);
    table->PutBoolean("Roller intaking?", state.isIndexIntake);
}

void Indexer::assignOutputs()
{
    if(state.inHandoff)
    {
        handoff();
    }
    if(state.isIndexIntake){
        rollerIntake();
    }
}

void Indexer::handoff()
{
    NoteHandoffMotor.setPower(maxHandoffSpeed);
}

void Indexer::rollerIntake()
{
    ITBRollerMotor.setPower(maxRollerSpeed);
}

double Indexer::getHandOffSpeed()
{
    return NoteHandoffMotor.getSpeed();
}

double Indexer::getRollerSpeed()
{
    return ITBRollerMotor.getSpeed();
}

void Indexer::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "handoffMotorSpeed",
        [this] {return getHandOffSpeed();},
        nullptr
    );

    builder.AddDoubleProperty(
        "rollerMotorSpeed",
        [this] {return getRollerSpeed();},
        nullptr
    );

    builder.AddBooleanProperty(
        "inHandoff",
        [this] {return state.inHandoff;},
        nullptr
    );

    builder.AddBooleanProperty(
        "rollerIntake",
        [this] {return state.isIndexIntake;},
        nullptr
    );
}