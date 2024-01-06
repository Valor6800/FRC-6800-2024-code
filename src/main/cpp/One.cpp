#include <iostream>
#include "One.h"

#define DEFAULT_MOTOR1_SPD 0.3
#define DEFAULT_DIRECTION1 true
#define Motor1_Buttons "Press left bumper to run motors"
#define CAN_BUS ""

One::One(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "One"),
    followMotors(CANIDs::MOTOR1, valor::NeutralMode::Brake, DEFAULT_DIRECTION1, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

One::~One()
{

}

void One::resetState()
{
    state.oneState = DISABLED;
}

void One::init()
{
    state.motor1Speed = DEFAULT_MOTOR1_SPD;

    followMotors.setupFollower(CANIDs::MOTOR2, DEFAULT_DIRECTION1);

    table->PutNumber("Motor1 Speed", state.motor1Speed);
    table->PutBoolean("Motor1 Direction", state.direction1);
    table->PutString("Motor1 Buttons", Motor1_Buttons);
    
    resetState();
}

void One::assessInputs()
{
    if(driverGamepad->GetLeftBumper()){
        state.oneState = ACTIVE;
    }else{
        state.oneState = DISABLED;
    }
}

void One::analyzeDashboard()
{
    state.motor1Speed = table->GetNumber("Motor1 Speed", DEFAULT_MOTOR1_SPD);
    state.direction1 = table->GetNumber("Motor1 Direction", DEFAULT_DIRECTION1);
}

void One::assignOutputs()
{
    if(state.oneState == DISABLED){
        followMotors.setPower(0);
    } else if(state.oneState == ACTIVE){
        followMotors.setPower((state.direction1 ? 1 : -1) * DEFAULT_MOTOR1_SPD);
    }
}

void One::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Motor1 State",
        [this]{return state.oneState;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Motor1 Speed",
        [this]{return state.motor1Speed;},
        nullptr
    );
    builder.AddBooleanProperty(
        "Motor1 Direction",
        [this]{return state.direction1;},
        nullptr
    );
}