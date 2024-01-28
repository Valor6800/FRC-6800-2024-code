#include <iostream>
#include "DriveChar.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "SysID"),
    frontLeftMotor(2,valor::NeutralMode::Brake, true, ""),
    frontRightMotor(4,valor::NeutralMode::Brake, false, ""),
    backLeftMotor(6,valor::NeutralMode::Brake, false, ""),
    backRightMotor(8,valor::NeutralMode::Brake, false, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void DriveChar::init()
{
    voltageCommand = 0.0;
    table->PutNumber("Voltage", voltageCommand);
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
        state.testType = DYNAMIC_F;
    } else if (operatorGamepad->GetAButtonPressed()){
        state.testType = DYNAMIC_R; 
    } else if (operatorGamepad->DPadUp()){
        state.testType = QUASISTATIC_F;
    } else if (operatorGamepad->DPadDown()){
        state.testType = QUASISTATIC_R;
    } else if (operatorGamepad->GetXButtonPressed()){
        state.testType = NO_MOVE;
        voltageCommand = 0.0;
    }
}

void DriveChar::analyzeDashboard()
{

}

void DriveChar::assignOutputs()
{
    if (state.testType == DYNAMIC_F || state.testType == DYNAMIC_R){
        motorVoltage = 7;
    } else if (state.testType == QUASISTATIC_F || state.testType == QUASISTATIC_R){
        voltageCommand += 1.0/50.0;
        motorVoltage = voltageCommand;
    } else if (state.testType == NO_MOVE){
        motorVoltage = 0.0;
    } else {
        motorVoltage = 0.0;
    }

    if (state.testType == DYNAMIC_F || state.testType == QUASISTATIC_F){
        frontLeftMotor.setInversion(true);
        frontRightMotor.setInversion(false);
        backLeftMotor.setInversion(false);
        backRightMotor.setInversion(false);
    }else if (state.testType == DYNAMIC_R || state.testType == QUASISTATIC_R){
        frontLeftMotor.setInversion(false);
        frontRightMotor.setInversion(true);
        backLeftMotor.setInversion(true);
        backRightMotor.setInversion(true);
    }

    frontLeftMotor.setVoltage(motorVoltage);
    frontRightMotor.setVoltage(motorVoltage);
    backLeftMotor.setVoltage(motorVoltage);
    backRightMotor.setVoltage(motorVoltage);
    
    if (state.testType == QUASISTATIC_F){
        table->PutString("State", "quasistatic-forward");
    } else if (state.testType == DYNAMIC_F){
        table->PutString("State", "dynamic-forward");    
    } else if (state.testType == QUASISTATIC_R){
        table->PutString("State", "quasistatic-reverse");
    } else if (state.testType == DYNAMIC_R){
        table->PutString("State", "dynamic-reverse");   
    }
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    // builder.AddStringProperty(
    //     "State",
    //     [this] 
    //     {
    //         if (state.testType == QUASISTATIC_F){
    //             return "quasistatic-forward";
    //         } else if (state.testType == DYNAMIC_F){
    //             return "dynamic-forward";
    //         } else if (state.testType == QUASISTATIC_R){
    //             return "quasistatic-reverse";
    //         } else if (state.testType == DYNAMIC_R){
    //             return "dynamic-reverse";
    //         } else{
    //             return "";
    //         }
    //     },
    //     nullptr);
}
