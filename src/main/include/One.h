#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class One : public valor::BaseSubsystem
{
public:
    One(frc::TimedRobot *robot);

    ~One();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder &builder) override;

    enum OneStates{
        DISABLED,
        ACTIVE
    };

    struct x
    {
        OneStates oneState;

        double motor1Speed;
        bool direction1;
    }state;

private:
    valor::NeoController followMotors;
};