#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeoController.h"

#define CLIMBER_K_VEL 0.0f
#define CLIMBER_K_ACC_MUL 0.0f
#define CLIMBER_K_F 0.0f
#define CLIMBER_K_P 0.0f
#define CLIMBER_K_I 0.0f
#define CLIMBER_K_D 0.0f
#define CLIMBER_K_ERROR 0.0f

#define CLIMBER_GEAR_RATIO 2.0f
#define CLIMBER_FORWARD_LIMIT 3.5f
#define CLIMBER_REVERSE_LIMIT 0.0f

#define CLIMBER_EXTENDED_POS 7.0f
#define CLIMBER_RESTING_POS 0.0f

Climber::Climber(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Climber"),
    RightClimbMotor(CANIDs::RIGHT_CLIMBER, valor::NeutralMode::Brake, false),
    LeftClimbMotor(CANIDs::LEFT_CLIMBER, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

Climber::~Climber()
{
}

void Climber::resetState()
{
    state.isExtended = false;
    state.isInClimb = false;
}

void Climber::init()
{
    climberPID.velocity = CLIMBER_K_VEL;
    climberPID.acceleration = CLIMBER_K_ACC_MUL;
    climberPID.F = CLIMBER_K_F;
    climberPID.P = CLIMBER_K_P;
    climberPID.I = CLIMBER_K_I;
    climberPID.D = CLIMBER_K_D;
    climberPID.error = CLIMBER_K_ERROR;

    RightClimbMotor.setConversion(1.0 / CLIMBER_GEAR_RATIO * 360);
    RightClimbMotor.setForwardLimit(CLIMBER_FORWARD_LIMIT);
    RightClimbMotor.setReverseLimit(CLIMBER_REVERSE_LIMIT);
    RightClimbMotor.setPIDF(climberPID, 0);

    LeftClimbMotor.setConversion(1.0 / CLIMBER_GEAR_RATIO * 360);
    LeftClimbMotor.setForwardLimit(CLIMBER_FORWARD_LIMIT);
    LeftClimbMotor.setReverseLimit(CLIMBER_REVERSE_LIMIT);
    LeftClimbMotor.setPIDF(climberPID, 0);

    resetState();

    table->PutBoolean("Are we fully extended?", state.isExtended);
    table->PutBoolean("Are we climbing?", state.isInClimb);
}

void Climber::assessInputs()
{
    if(operatorGamepad->rightStickYActive() && operatorGamepad->GetRightBumperPressed())
    {
        state.isExtended = true; 
        state.isInClimb = true;
    }

    else if(operatorGamepad->rightStickYActive())
    {
        state.isExtended = true;
    }
}

void Climber::analyzeDashboard()
{
    table->PutBoolean("Are we extended?", state.isExtended);
    table->PutBoolean("Are we climbing?", state.isInClimb);
}

void Climber::assignOutputs()
{
    if(state.isExtended && state.isInClimb)
    {
            climb();
    }

    else if(state.isExtended)
    {
            extend();
    }
}

void Climber::extend()
{
    if(state.isExtended)
    {
        RightClimbMotor.setPosition(CLIMBER_EXTENDED_POS);
        LeftClimbMotor.setPosition(CLIMBER_EXTENDED_POS);
    }
    else if(!state.isExtended)
    {
        RightClimbMotor.setPosition(CLIMBER_RESTING_POS);
        LeftClimbMotor.setPosition(CLIMBER_RESTING_POS);
    }
}

void Climber::climb()
{
    extend();
    // add the addition climbing stuff
}

double Climber::getClimberSpeed()
{
    double e = 0;
    double r = RightClimbMotor.getSpeed();
    double l = LeftClimbMotor.getSpeed();
    if(r > l)
    {
        e = r - l;
    }
    else if(l > r)
    {
        e = l - r;
    }
    else
    {
        e = 0;
    }
    if(e < 0.2)
    {
        return r;
    }
    else
    {
        return -1;
    }
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "climberSpeed",
        [this] {return getClimberSpeed();},
        nullptr
    );

    builder.AddBooleanProperty(
        "isExtended",
        [this] {return state.isExtended;},
        nullptr
    );

    builder.AddBooleanProperty(
        "isClimbing",
        [this] {return state.isInClimb;},
        nullptr
    );
}