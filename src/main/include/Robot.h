#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "valkyrie/Gamepad.h"

#include "One.h"
#include "Three.h"
#include "Four.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include <frc/livewindow/LiveWindow.h>

#include <fstream>

class Robot : public frc::TimedRobot {
    public:
        Robot();

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::Command * autoCommand = nullptr;

        One oneAndTwo;
        Three three;
        Four four;

        std::ofstream outfile;
};
