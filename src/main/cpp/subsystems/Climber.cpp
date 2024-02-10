#include <iostream>
#include "subsystems/Climber.h"
#include "Constants.h"

#include "valkyrie/sensors/DebounceSensor.h"

#define MAX_SPEED 0

Climber::Climber(frc::TimedRobot *_robot) : 
    valor::BaseSubsystem(_robot, "Climber"), 
    climbMotor(CANIDs::LEFT_CLIMBER, valor::NeutralMode::Brake, false),
    hallE(new frc::DigitalInput(CANIDs::HALL_EFFECT)),
    debounce(_robot, "Climber")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
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
    climbMotor.setForwardLimit(20);
    climbMotor.setReverseLimit(0);
    resetState();
}

void Climber::assessInputs()
{
    if (!operatorGamepad) return;
    
    if (operatorGamepad->DPadUp()){
        state.climbState == AUTO_CLIMB;
    } else if (operatorGamepad->rightStickYActive()){
        state.climbState = ACTIVE;
    } else {
        state.climbState = DISABLED;
    }
}

void Climber::analyzeDashboard()
{
    if(hallE->Get() == true){
        state.zeroState = ZERO;
    }
    table->PutBoolean("Zero", state.zeroState == ZERO);
}

void Climber::assignOutputs()
{
    if (state.zeroState != ZERO){
        state.elevSpeed = 0;
    } else if (state.climbState == AUTO_CLIMB){
        //auto climb sequence
    } else if (state.climbState == ACTIVE){
        state.elevSpeed = operatorGamepad->rightStickY() * MAX_SPEED;
    } else if (state.climbState == DISABLED){
        state.elevSpeed = 0;
    }
    climbMotor.setSpeed(state.elevSpeed);
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    
}