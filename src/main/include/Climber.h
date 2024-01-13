#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/TimedRobot.h>


class Climber : public valor::BaseSubsystem
{
public:
    Climber(frc::TimedRobot *robot);
    double target_pose;
    double speed_multiplier;
    bool KILL;

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    void InitSendable(wpi::SendableBuilder& builder);


private:
    valor::NeoController climbMotor;

    
};