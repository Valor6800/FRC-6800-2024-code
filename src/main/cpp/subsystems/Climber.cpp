#include "subsystems/Climber.h"

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"), 
climbMotor(9, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Climber::init()
{
    target_pose = 0;
    climbMotor.setEncoderPosition(0);
    climbMotor.setupFollower(10, false);
    speed_multiplier = 1.00;
    climbMotor.setForwardLimit(20);
    climbMotor.setReverseLimit(0);

    table->PutNumber("speed multiplier", speed_multiplier);
}

void Climber::resetState()
{
    //set disabled state
}

void Climber::assessInputs()
{
    if (!operatorGamepad) return;
    target_pose = operatorGamepad->rightStickY(3) * speed_multiplier;
}

void Climber::analyzeDashboard()
{
    table->PutNumber("target pose", target_pose); 
    // speed_multiplier = table->GetNumber("speed multiplier", .05);  
}

void Climber::assignOutputs()
{
    climbMotor.setPower(target_pose);
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{

}