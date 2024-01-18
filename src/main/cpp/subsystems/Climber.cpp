#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeoController.h"

#define CLIMBER_K_VEL 0.0f
#define CLIMBER_K_ACC_MUL 0.0f
#define CLIMBER_K_F 0.0f
#define CLIMBER_K_P 0.0f
#define CLIMBER_K_I 0.0f
#define CLIMBER_K_D 0.0f
#define CLIMBER_K_ERROR 0.0f


Climber::Climber(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Climber"),
    RightClimbMotor(CANIDs::RIGHT_CLIMBER, valor::NeutralMode::Brake, false),
    LeftClimbMotor(CANIDs::LEFT_CLIMBER, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

Climber::~Climber()
{

}

void Climber::init()
{
    climberPID.velocity = CLIMBER_K_VEL;
    climberPID.acceleration = CLIMBER_K_ACC_MUL;
    climberPID.F = CLIMBER_K_F;
    climberPID.P = CLIMBER_K_P;
    climberPID.I = CLIMBER_K_I;
    climberPID.D = CLIMBER_K_D;
    climberPID.error = CLIMBER_K_ERROR;
}