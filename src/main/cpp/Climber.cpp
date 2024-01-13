#include "Climber.h"

Climber::Climber(frc::TimedRobot *_robot) : 
valor::BaseSubsystem(_robot, "Climber"), 
climbMotor(9, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    target_pose = 0;
    climbMotor.setEncoderPosition(0);
    climbMotor.setupFollower(10, false);
    KILL = false;
    speed_multiplier = .25;
    table->PutNumber("speed multiplier", speed_multiplier);
    climbMotor.setForwardLimit(20);
    climbMotor.setReverseLimit(0);

}
\
void Climber::assessInputs()
{
    if (!operatorGamepad) return;
    if (operatorGamepad->GetBButtonPressed()) KILL = true;
    
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