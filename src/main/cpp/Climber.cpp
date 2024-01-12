#include "Climber.h"

Climber::Climber(frc::TimedRobot *_robot) : 
valor::BaseSubsystem(_robot, "Climber"), 
climbMotor(13, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    target_pose = 0;
    climbMotor.setEncoderPosition(target_pose);
    KILL = false;
}

void Climber::assessInputs()
{
    if (!driverGamepad) return;
    if (driverGamepad->GetBButtonPressed()) KILL = !KILL;
    
    target_pose += driverGamepad->rightStickY(3);
}

void Climber::analyzeDashboard()
{
    table->PutNumber("target pose", target_pose);   
}

void Climber::assignOutputs()
{
    if (!KILL) climbMotor.setPosition(target_pose);
    else climbMotor.setPower(0);
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{

}