#pragma once

#include "Constants.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/sensors/DebounceSensor.h"

#include "frc/DigitalInput.h"

#include <frc/TimedRobot.h>


class Climber : public valor::BaseSubsystem
{
public:
    Climber(frc::TimedRobot *robot);

    void init() override;
    void resetState() override;
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void InitSendable(wpi::SendableBuilder& builder) override;

    enum CLIMB_STATE
    {
        DISABLED,
        ACTIVE,
        AUTO_CLIMB
    };

    enum ZERO_STATE
    {
        NOT_ZERO,
        ZERO
    };

    struct x {
        CLIMB_STATE climbState;
        ZERO_STATE zeroState;

        double elevSpeed;

    }state;

private:
    valor::NeoController climbMotor;

    frc::DigitalInput* hallE;
    valor::DebounceSensor debounce;
};