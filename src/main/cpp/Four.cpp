#include <iostream>
#include "Four.h"

#define DEFAULT_MOTOR4_SPD 0.3
#define DEFAULT_DIRECTION4 true
#define Motor4_Buttons "Press right trigger to run motor 4"


Four::Four(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Four"),
    fourMotor(CANIDs::MOTOR4, valor::NeutralMode::Coast, DEFAULT_DIRECTION4, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Four::~Four()
{

}

void Four::resetState()
{
    state.fourState = DISABLED;
}

void Four::init()
{
    state.motor4Speed = DEFAULT_MOTOR4_SPD;
    state.direction4 = DEFAULT_DIRECTION4;

    table->PutNumber("Motor4 Speed", state.motor4Speed);
    table->PutBoolean("Motor4 Direction", state.direction4);
    table->PutString("Motor4 Buttons", Motor4_Buttons);

    resetState();
}

void Four::assessInputs()
{
    if(driverGamepad->rightTriggerActive()){
        state.fourState = ACTIVE;
    } else {
        state.fourState = DISABLED;
    }
}

void Four::analyzeDashboard()
{
    state.motor4Speed = table->GetNumber("Motor4 Speed", DEFAULT_MOTOR4_SPD);
    state.direction4 = table->GetBoolean("Motor4 Direction", DEFAULT_DIRECTION4);
}

void Four::assignOutputs()
{
    if(state.fourState == DISABLED){
        fourMotor.setPower(0);
    }else if(state.fourState == ACTIVE){
        fourMotor.setPower((state.direction4 ? 1 : -1) * state.motor4Speed);
    }
}

void Four::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Motor4 State",
        [this]{return state.fourState;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Motor4 Speed",
        [this]{return state.motor4Speed;},
        nullptr
    );
    builder.AddBooleanProperty(
        "Motor4 Direction",
        [this]{return state.direction4;},
        nullptr
    );
}