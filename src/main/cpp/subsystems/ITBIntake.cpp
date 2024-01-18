#include "subsystems/ITBIntake.h"
// #include "frc/DriverStation.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

ITBIntake::ITBIntake(frc::TimedRobot *_robot) : 
    valor::BaseSubsystem(_robot, "ITBIntake"),
    ITBIntakeRollerMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

ITBIntake::~ITBIntake()
{

}

void ITBIntake::init()
{

}
