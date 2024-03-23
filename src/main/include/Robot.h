// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <subsystems/Shooter.h>

#include <fstream>

#include "Constants.h"
#include "Drivetrain.h"
#include "frc/AnalogTrigger.h"
#include "frc/AnalogTriggerOutput.h"
#include "frc/DigitalInput.h"
#include "subsystems/Feeder.h"
#include "subsystems/Shooter.h"
#include "valkyrie/Auto.h"
#include "valkyrie/Gamepad.h"
#include "valkyrie/sensors/CANdleSensor.h"

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

    frc2::CommandPtr autoCommand = frc2::cmd::Sequence();

    valor::CANdleSensor leds;
    Drivetrain drivetrain;
    valor::Auto valorAuto;
    frc::AnalogTrigger feederBeamBreak;
    frc::AnalogTrigger feederBeamBreak2;
    frc::AnalogTrigger intakeBeamBreak;

    Shooter shooter;
    Feeder feeder;
};
