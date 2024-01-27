#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "Drivetrain.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class DriveChar : public valor::BaseSubsystem
{
public: 
    DriveChar(frc::TimedRobot *_robot);

    void resetState() override;
    
    void init() override;
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void upVoltage();
    double voltageCommand;
    double motorVoltage;
    double position;
    double velocity;
    double voltage;

    void InitSendable(wpi::SendableBuilder &builder) override;

    enum TestType{
        NO_MOVE,
        QUASISTATIC,
        DYNAMIC
    };

    struct x
    {
        TestType testType;

    }state;

private:
    valor::NeoController frontLeftMotor;
    valor::NeoController frontRightMotor;
    valor::NeoController backLeftMotor;
    valor::NeoController backRightMotor;
};
