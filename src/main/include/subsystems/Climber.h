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

    void init() override;
    void resetState() override;
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void InitSendable(wpi::SendableBuilder& builder) override;


private:
    valor::NeoController climbMotor;
};