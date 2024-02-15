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
    void setClimbPID();
    void InitSendable(wpi::SendableBuilder& builder) override;

    enum CLIMB_STATE
    {
        DISABLED,
        UP,
        DOWN,
        ACTIVE
    };

    enum ZERO_STATE
    {
        NOT_ZERO,
        ZERO,
        ZEROING
    };

    struct x {
        CLIMB_STATE climbState;
        ZERO_STATE zeroState;
    }state;

private:
    valor::NeoController climbMotor;

    frc::DigitalInput* hallE;
    valor::DebounceSensor debounce;

    frc2::SequentialCommandGroup zeroingSequence;
};