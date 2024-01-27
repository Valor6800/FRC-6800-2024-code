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
    DriveChar(frc::TimedRobot *_robot, Drivetrain* _drive);

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
        QUASISTATIC,
        DYNAMIC,
        NO_MOVE
    };

    struct x
    {
        TestType testType;

    }state;

private:
    Drivetrain* drive;
};
