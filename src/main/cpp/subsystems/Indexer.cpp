#include "subsystems/Indexer.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

Indexer::Indexer(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Indexer"),
    NoteHandoffMotor(CANIDs::HANDOFF_CONTROLLER, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Indexer::~Indexer()
{
    
}

void Indexer::init()
{

}