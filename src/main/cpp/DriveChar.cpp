#include <iostream>
#include "DriveChar.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "SysID"),
    frontLeftMotor(2,valor::NeutralMode::Brake,true,""),
    frontRightMotor(4,valor::NeutralMode::Brake,false,""),
    backLeftMotor(6,valor::NeutralMode::Brake,false,""),
    backRightMotor(8,valor::NeutralMode::Brake,false,"")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void DriveChar::init()
{
    voltageCommand = 0.0;
    table->PutNumber("Voltage", voltageCommand);
    table->PutNumber("State", state.testType);
}

void DriveChar::resetState()
{
    state.testType = NO_MOVE;
    voltageCommand = 0.0;
}

void DriveChar::assessInputs()
{
    if (!operatorGamepad || !driverGamepad)
        return;

    if (operatorGamepad->GetYButtonPressed()){
        state.testType = DYNAMIC;
    } else if (operatorGamepad->GetAButtonPressed()){
        state.testType = QUASISTATIC;
    } else if (operatorGamepad->GetXButtonPressed()){
        state.testType = NO_MOVE;
    }

    table->PutNumber("State", state.testType);
}

void DriveChar::analyzeDashboard()
{

}

void DriveChar::assignOutputs()
{
    if (state.testType == DYNAMIC){
        motorVoltage = 7;
    } else if (state.testType == QUASISTATIC){
        voltageCommand += 1.0/50.0;
        table->PutNumber("Voltage", voltageCommand);
        motorVoltage = voltageCommand;
    } else if (state.testType == NO_MOVE){
        motorVoltage = 0.0;
    } else {
        motorVoltage = 0.0;
    }
    frontLeftMotor.setVoltage(motorVoltage);
    frontRightMotor.setVoltage(motorVoltage);
    backLeftMotor.setVoltage(motorVoltage);
    backRightMotor.setVoltage(motorVoltage);
    // position = drive->getPose_m().X().to<double>();
    // velocity = motorVoltage / 12.0 * drive->getDriveMaxSpeed();
    // drive->drive(units::velocity::meters_per_second_t(velocity), 0.0_mps, units::angular_velocity::radians_per_second_t(0.0), true);
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "State",
        [this] {return state.testType;},
        nullptr);
}
