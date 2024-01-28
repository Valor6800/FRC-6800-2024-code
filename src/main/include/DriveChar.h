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

    double voltageCommand;
    double motorVoltage;

    void InitSendable(wpi::SendableBuilder &builder) override;

    enum TestType{
        NO_MOVE,
        QUASISTATIC_F,
        DYNAMIC_F,
        QUASISTATIC_R,
        DYNAMIC_R
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
