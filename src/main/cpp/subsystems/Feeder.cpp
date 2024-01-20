#include "subsystems/Feeder.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define Feeder_GEAR_RATIO 0.0f
#define ITB_ROLLER_GEAR_RATIO 2.0f

Feeder::Feeder(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Feeder"),
    NoteHandoffMotor(CANIDs::HANDOFF_CONTROLLER, valor::NeutralMode::Brake, false),
    ITBRollerMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Feeder::~Feeder()
{   
}

void Feeder::resetState()
{
    state.inHandoff = false;
    state.isIndexIntake = false;
}

void Feeder::init()
{
    NoteHandoffMotor.setConversion(1.0 / Feeder_GEAR_RATIO * 360);
    ITBRollerMotor.setConversion(1.0 / ITB_ROLLER_GEAR_RATIO * 360);

    resetState();

    table->PutBoolean("In Handoff?", state.inHandoff);
    table->PutBoolean("Roller Intaking?", state.isIndexIntake);
}

void Feeder::assessInputs()
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

void Feeder::analyzeDashboard()
{
    table->PutBoolean("In handoff?", state.inHandoff);
    table->PutBoolean("Roller intaking?", state.isIndexIntake);
}

void Feeder::assignOutputs()
{
    if(state.inHandoff)
    {
        handoff();
    }
    if(state.isIndexIntake){
        rollerIntake();
    }
}

void Feeder::handoff()
{
    NoteHandoffMotor.setPower(maxHandoffSpeed);
}

void Feeder::rollerIntake()
{
    ITBRollerMotor.setPower(maxRollerSpeed);
}

double Feeder::getHandOffSpeed()
{
    return NoteHandoffMotor.getSpeed();
}

double Feeder::getRollerSpeed()
{
    return ITBRollerMotor.getSpeed();
}

void Feeder::InitSendable(wpi::SendableBuilder& builder)
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