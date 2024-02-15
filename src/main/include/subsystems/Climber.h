#pragma once

#include "Constants.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/sensors/DebounceSensor.h"

#include "frc/DigitalInput.h"

#include <frc/TimedRobot.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

class Climber : public valor::BaseSubsystem
{
public:
    Climber(frc::TimedRobot *robot);

    ~Climber();

    void init() override;
    void resetState() override;
    void assessInputs() override;
    void analyzeDashboard() override;
    void climbCommands();
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
        ZERO,
        ZEROING
    };

    enum AUTO_CLIMB_STATE
    {
        DISABLED_CLIMBER,
        UP_CLIMBER,
        DOWN_CLIMBER
    };

    struct x {
        CLIMB_STATE climbState;
        ZERO_STATE zeroState;
        AUTO_CLIMB_STATE autoClimbState;

        double elevSpeed;

    }state;

private:
    valor::NeoController climbMotor;

    frc::DigitalInput* hallE;
    valor::DebounceSensor debounce;

    frc2::SequentialCommandGroup autoClimbSequence;
    frc2::SequentialCommandGroup zeroingSequence;
};