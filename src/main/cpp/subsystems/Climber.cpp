#include <iostream>
#include "subsystems/Climber.h"
#include "Constants.h"

#include "valkyrie/sensors/DebounceSensor.h"

#define MAX_CLIMB_SPEED 0 //placeholder

#define UP_CLIMB_TARGET 0 //placeholder
#define DOWN_CLIMB_TARGET 0 //placeholder

#define UP_CLIMB_SPEED 0 //placeholder
#define DOWN_CLIMB_SPEED 0 //placeholder

Climber::Climber(frc::TimedRobot *_robot) : 
    valor::BaseSubsystem(_robot, "Climber"), 
    climbMotor(CANIDs::LEFT_CLIMBER, valor::NeutralMode::Brake, false),
    hallE(new frc::DigitalInput(CANIDs::HALL_EFFECT)),
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
    climbCommands();

    resetState();
}

void Climber::climbCommands(){
    frc2::FunctionalCommand upClimb(
        [this]() {
            state.autoClimbState = Climber::AUTO_CLIMB_STATE::UP_CLIMBER;
        },
        [this]() {},
        [this](bool) {
            state.autoClimbState = Climber::AUTO_CLIMB_STATE::DISABLED_CLIMBER;
        },
        [this](){
            return climbMotor.getPosition() >= UP_CLIMB_TARGET;
        },
        {}
    );

    frc2::FunctionalCommand downClimb(
        [this]() {
            state.autoClimbState = Climber::AUTO_CLIMB_STATE::DOWN_CLIMBER;
        },
        [this]() {},
        [this](bool) {
            state.autoClimbState = Climber::AUTO_CLIMB_STATE::DISABLED_CLIMBER;
        },
        [this](){
            return climbMotor.getPosition() <= DOWN_CLIMB_TARGET;
        },
        {}
    );
    autoClimbSequence.AddCommands(upClimb, downClimb);
}

void Climber::assessInputs()
{
    if (!operatorGamepad) return;


    if(operatorGamepad->rightStickYActive() && autoClimbSequence.IsScheduled()){
        autoClimbSequence.Cancel();
    }

    if(!autoClimbSequence.IsScheduled() )
    {
        state.autoClimbState = AUTO_CLIMB_STATE::DISABLED_CLIMBER;
    }
    
    if (operatorGamepad->DPadUp()){
        state.climbState = AUTO_CLIMB;
        autoClimbSequence.Schedule();
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
        climbMotor.setSpeed(0);
    } else if (state.climbState == AUTO_CLIMB){

        if (state.autoClimbState == UP_CLIMBER){
            climbMotor.setSpeed(UP_CLIMB_SPEED);
        } else if (state.autoClimbState == DOWN_CLIMB_SPEED){
            climbMotor.setSpeed(DOWN_CLIMB_SPEED);
        } else if (state.autoClimbState == DISABLED_CLIMBER){
            climbMotor.setSpeed(0);
        }

    } else if (state.climbState == ACTIVE){
        climbMotor.setSpeed(operatorGamepad->rightStickY() * MAX_CLIMB_SPEED);
    } else if (state.climbState == DISABLED){
        climbMotor.setSpeed(0);
    }
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    
}