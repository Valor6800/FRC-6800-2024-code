#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "frc/AnalogTrigger.h"
#include "frc/AnalogTriggerOutput.h"
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"
#include "subsystems/Shooter.h"
#include "valkyrie/sensors/CANdleSensor.h"

#include "valkyrie/Auto.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include <frc/livewindow/LiveWindow.h>

#include <subsystems/Shooter.h>
#include "subsystems/Feeder.h"
#include "subsystems/Climber.h"

#include <fstream>
#include "frc/DigitalInput.h"

class Robot : public frc::TimedRobot {
    public:
        Robot();

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        void AutonomousExit() override;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        std::vector<frc2::CommandPtr> autoCommands;

        valor::CANdleSensor leds;
        Drivetrain drivetrain;
        valor::Auto valorAuto;
        frc::AnalogTrigger feederBeamBreak;
        frc::AnalogTrigger stageBeamBreak;
        
        Shooter shooter;
        Feeder feeder;
        Climber climber;
};
