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
    climbMotor.setupFollower(CANIDs::RIGHT_CLIMBER, true);
    climbMotor.setEncoderPosition(0);
    climbMotor.setForwardLimit(48); //placeholder
    // climbMotor.setReverseLimit(0);
    climbMotor.setVoltageCompensation(12);
    climbMotor.setMaxCurrent(100);
    setClimbPID();

    resetState();

    table->PutBoolean("Manual Climber", false);
}

void Climber::assessInputs()
{
    if (!operatorGamepad || !operatorGamepad->IsConnected()) return;

    state.manualClimber = operatorGamepad->leftStickY(2);

    if (state.zeroState == NOT_ZERO && !hallE->Get()){
        state.zeroState = ZERO_STATE::ZEROING;
    } else if (operatorGamepad->DPadUp()){
        state.climbState = UP;
    } else if (operatorGamepad->DPadDown()){
        state.climbState = DOWN;
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
    // if (table->GetBoolean("Manual Climber", false)) {
    //     state.climbState = CLIMB_STATE::MANUAL;
    // }
}

void Climber::assignOutputs()
{
    // if (state.climbState == CLIMB_STATE::MANUAL) {
        climbMotor.setPower(state.manualClimber);
    // } else if (state.zeroState == ZEROING){
    //     climbMotor.setSpeed(ZERO_SPEED);
    // } else if (state.climbState == UP){
    //     climbMotor.setPosition(UP_POSE);
    // } else if (state.climbState == DOWN){
    //     climbMotor.setPosition(DOWN_POSE);
    // } else if (state.climbState == ACTIVE){
    //     climbMotor.setSpeed(operatorGamepad->rightStickY() * MAX_CLIMB_SPEED);
    // } else {
    //     climbMotor.setSpeed(0);
    // }
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

    builder.AddIntegerProperty(
        "Zero state",
        [this] {return state.zeroState;},
        nullptr
    );
    builder.AddIntegerProperty(
        "Climb state",
        [this] {return state.climbState;},
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