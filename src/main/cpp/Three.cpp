#include <iostream>
#include "Three.h"

#define DEFAULT_MOTOR3_SPD 0.3
#define DEFAULT_DIRECTION3 true
#define Motor3_Buttons "Press left trigger to run motor"

Three::Three(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Three"),
    threeMotor(CANIDs::MOTOR3, valor::NeutralMode::Coast, DEFAULT_DIRECTION3, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Three::~Three()
{

}

void Three::resetState()
{
    state.threeState = DISABLED;
}

void Three::init()
{
    state.motor3Speed = DEFAULT_MOTOR3_SPD;
    state.direction3 = DEFAULT_DIRECTION3;

    table->PutNumber("Motor3 Speed", state.motor3Speed);
    table->PutBoolean("Motor3 Direction", state.direction3);
    table->PutString("Motor3 Buttons", Motor3_Buttons);

    resetState();
}

void Three::assessInputs()
{
    if(driverGamepad->leftTriggerActive()){
        state.threeState = ACTIVE;
    } else {
        state.threeState = DISABLED;
    }
}

void Three::analyzeDashboard()
{
    state.motor3Speed = table->GetNumber("Motor3 Speed", DEFAULT_MOTOR3_SPD);
    state.direction3 = table->GetBoolean("Motor3 Direction", DEFAULT_DIRECTION3);
}

void Three::assignOutputs()
{
    if(state.threeState == DISABLED){
        threeMotor.setPower(0);
    } else if(state.threeState == ACTIVE){
        threeMotor.setPower((state.direction3 ? 1 : -1) * state.motor3Speed);
    }
}

void Three::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Motor3 State",
        [this]{return state.threeState;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Motor3 Speed",
        [this]{return state.motor3Speed;},
        nullptr
    );
    builder.AddBooleanProperty(
        "Motor3 Direction",
        [this]{return state.direction3;},
        nullptr
    );
    builder.AddStringProperty(
        "Motor3 Buttons",
        [this]{return Motor3_Buttons;},
        nullptr
    );
}