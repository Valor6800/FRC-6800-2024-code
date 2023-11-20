#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Three : public valor::BaseSubsystem
{
public:
    Three(frc::TimedRobot *robot);

    ~Three();

    void init() override;
    
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder &builder) override;

    enum ThreeStates{
        DISABLED,
        ACTIVE
    };

    struct x
    {
        ThreeStates threeState;

        double motor3Speed;
        bool direction3;
    }state;
    
private:
    valor::NeoController threeMotor;
};