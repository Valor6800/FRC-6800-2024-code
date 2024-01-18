#include "subsystems/OTBIntake.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

OTBIntake::OTBIntake(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "OTBIntake"),
    OTBIntakeRollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Brake, false),
    OTBDropDownMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

OTBIntake::~OTBIntake()
{

}

void OTBIntake::init()
{

}