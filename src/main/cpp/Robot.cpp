#include "Robot.h"
#include "frc/AnalogTriggerType.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/RobotController.h>
#include <Constants.h>

#include <ctime>

Robot::Robot() : 
    drivetrain(this),
    // valorAuto(), 
    beamBreak(AnalogPorts::BEAM_BREAK_PORT),
    climber(this),
    shooter(this, &climber,  &drivetrain),
    feeder(this, &beamBreak),
    leds(this, &feeder, &shooter)

{
    frc::TimedRobot();
    beamBreak.SetLimitsVoltage(4, 14);
}

void Robot::RobotInit() {

    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    shooter.setGamepads(&gamepadOperator, &gamepadDriver);
    shooter.resetState();

    feeder.setGamepads(&gamepadOperator, &gamepadDriver);
    feeder.resetState();

    climber.setGamepads(&gamepadOperator, &gamepadDriver);
    climber.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();

    // valorAuto.fillAutoList();

}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { }

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    drivetrain.resetState();
    drivetrain.state.matchStart = frc::Timer::GetFPGATimestamp().to<double>();
    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Brake);

    feeder.resetState();
    shooter.resetState();

    // autoCommand = valorAuto.getCurrentAuto();
    // autoCommand.Schedule();
}

void Robot::AutonomousExit() {

    drivetrain.state.xPose = true;
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {

    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Coast);

    shooter.resetState();
    feeder.resetState();

    // autoCommand.Cancel();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
