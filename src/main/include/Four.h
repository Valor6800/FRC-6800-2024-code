#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Four : public valor::BaseSubsystem
{
public:
    Four(frc::TimedRobot *robot);

    ~Four();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder &builder) override;

    enum FourStates{
        DISABLED,
        ACTIVE
    };

    struct x 
    {
        FourStates fourState;

        double motor4Speed;
        bool direction4;
    }state;

private:
    valor::NeoController fourMotor;
};