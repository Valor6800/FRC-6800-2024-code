#include <iostream>
#include "subsystems/Climber.h"
#include "Constants.h"

#include "valkyrie/sensors/DebounceSensor.h"

#define MAX_CLIMB_SPEED 0 //placeholder
#define MAX_CLIMB_ACCEL 0 //placeholder
#define CONVERSION 1 //placeholder

#define ZERO_TARGET 0 //placeholder
#define ZERO_SPEED 0 //placeholder

#define UP_POSE 0 //placeholder
#define DOWN_POSE 0 //placeholder

Climber::Climber(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Climber"), 
    climbMotor(CANIDs::LEFT_CLIMBER, valor::NeutralMode::Brake, false),
    hallE(new frc::DigitalInput(DIOPorts::HALL_EFFECT)),
    debounce(_robot, "Climber")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Climber::~Climber()
{
}

void Climber::resetState()
{
    //set disabled state
    state.climbState = DISABLED;
    state.zeroState = NOT_ZERO;
}

void Climber::init()
{
    climbMotor.setupFollower(CANIDs::RIGHT_CLIMBER, false);
    climbMotor.setEncoderPosition(0);
    climbMotor.setForwardLimit(20); //placeholder
    climbMotor.setReverseLimit(0);
    climbMotor.setVoltageCompensation(10);
    setClimbPID();
    climbCommands();

    resetState();
}

void Climber::climbCommands()
{
    frc2::FunctionalCommand zeroing(
        [this]() {
            state.zeroState = Climber::ZERO_STATE::ZEROING;
        },
        [this]() {},
        [this](bool) {
            state.zeroState = Climber::ZERO_STATE::ZERO;
        },
        [this]() {
            return climbMotor.getPosition() <= ZERO_TARGET || hallE->Get();
        },
        {}
    );
    zeroingSequence.AddCommands(zeroing);
}

void Climber::assessInputs()
{
    if (!operatorGamepad) return;
    if(!zeroingSequence.IsScheduled())
    {
        state.climbState = DISABLED;
    }

    if (state.zeroState == NOT_ZERO && !hallE->Get()){
        zeroingSequence.Schedule();
    } else if (operatorGamepad->DPadUp()){
        state.climbState = UP;
    } else if (operatorGamepad->DPadRight()){
        state.climbState = DOWN;
    } else if (operatorGamepad->rightStickYActive()){
        state.climbState = ACTIVE;
    } else {
        state.climbState = DISABLED;
    }
}

void Climber::analyzeDashboard()
{
    if(hallE->Get()){
        state.zeroState = ZERO;
    }
    table->PutBoolean("Zero", state.zeroState == ZERO);
}

void Climber::assignOutputs()
{
    if (state.zeroState == ZEROING){
        climbMotor.setSpeed(ZERO_SPEED);
    } else if (state.climbState == UP){
        climbMotor.setPosition(UP_POSE);
    } else if (state.climbState == DOWN){
        climbMotor.setPosition(DOWN_POSE);
    } else if (state.climbState == ACTIVE){
        climbMotor.setSpeed(operatorGamepad->rightStickY() * MAX_CLIMB_SPEED);
    } else if (state.climbState == DISABLED || state.zeroState == ZERO){
        climbMotor.setSpeed(0);
    } else{
        climbMotor.setSpeed(0);
    }
}

void Climber::setClimbPID()
{
    valor::PIDF climbPID;
    climbPID.maxVelocity = MAX_CLIMB_SPEED;
    climbPID.maxAcceleration = MAX_CLIMB_ACCEL;
    climbMotor.setConversion(CONVERSION);
    climbMotor.setPIDF(climbPID, 0);
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Is Zero",
        [this] {return state.zeroState == ZERO;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Climb Pose",
        [this] {return climbMotor.getPosition();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Target Up Pose",
        [this] {return UP_POSE;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Target Down Pose",
        [this] {return DOWN_POSE;},
        nullptr
    );
}