#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"
#include "valkyrie/Auto.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>
#include <subsystems/Shooter.h>

#include <frc/livewindow/LiveWindow.h>
#include "subsystems/Intake.h"

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

        // A check for Alpha was specifically chosen because in the case the check brakes it defaults to comp numbers
        bool isAlpha;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::CommandPtr autoCommand = frc2::cmd::Sequence();

        Drivetrain drivetrain;
        valor::Auto valorAuto;
        frc::DigitalInput beamBreak;  
          
        Shooter shooter;
        Intake intake;

        std::ofstream outfile;        
};
