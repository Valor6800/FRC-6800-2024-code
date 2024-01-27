#include <iostream>
#include "DriveChar.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot, Drivetrain *_drive) : valor::BaseSubsystem(_robot, "SysID"),
    drive(_drive)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void DriveChar::init()
{
    voltageCommand = 0.0;
    table->PutNumber("Voltage", voltageCommand);
    table->PutString("State", "NO MOVE");
}

void DriveChar::assessInputs()
{
    if (operatorGamepad->GetYButtonPressed()){
        state.testType = QUASISTATIC;
    } else if (operatorGamepad->GetAButtonPressed()){
        state.testType = DYNAMIC;
    } else {
        state.testType = NO_MOVE;
    }
}

void DriveChar::analyzeDashboard()
{

}

void DriveChar::assignOutputs()
{
    if (operatorGamepad->GetBButtonPressed()){
        voltageCommand++;
        table->PutNumber("Voltage Command", voltageCommand);
    }

    if (state.testType == QUASISTATIC){
        motorVoltage = voltageCommand * frc::Timer::GetFPGATimestamp().to<double>();
    } else if (state.testType == DYNAMIC){
        motorVoltage = voltageCommand;
    } else if (state.testType == NO_MOVE){
        motorVoltage = 0.0;
    } else{
        motorVoltage = 0.0;
    }

    position = drive->getPose_m().X().to<double>();
    velocity = voltageCommand / 12.0 * drive->getDriveMaxSpeed();
    drive->drive(units::velocity::meters_per_second_t(velocity), 0.0_mps, units::angular_velocity::radians_per_second_t(0.0), true);
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "X Pose", 
        [this] {return position;}, 
        nullptr
    );
    builder.AddDoubleProperty(
        "Velocity",
        [this] {return velocity;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Velocity",
        [this] {return voltageCommand;},
        nullptr
    );
}
