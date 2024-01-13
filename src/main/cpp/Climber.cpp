#include "Climber.h"

Climber::Climber(frc::TimedRobot *_robot) : 
valor::BaseSubsystem(_robot, "Climber"), 
climbMotor(9, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    target_pose = 0;
    climbMotor.setEncoderPosition(target_pose);
    climbMotor.setupFollower(10, false);
    KILL = false;
    speed_multiplier = .001;
    table->PutNumber("speed multiplier", speed_multiplier);
}

void Climber::assessInputs()
{
    if (!operatorGamepad) return;
    if (operatorGamepad->GetBButtonPressed()) KILL = !KILL;
    
    target_pose += operatorGamepad->rightStickY(3) * speed_multiplier;
}

void Climber::analyzeDashboard()
{
    table->PutNumber("target pose", target_pose); 
    speed_multiplier = table->GetNumber("speed multiplier", .001);  
}

void Climber::assignOutputs()
{
    if (!KILL) climbMotor.setPosition(target_pose);
    else climbMotor.setPower(0);
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{

}