#include "subsystems/OTBIntake.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define OTB_ROLLER_GEAR_RATIO 4.0f
#define OTB_DROPDOWN_GEAR_RATIO 3.0f

#define OTB_DROPDOWN_POSITION 33.0f
#define OTB_INSIDE_POSITION 0.0f

OTBIntake::OTBIntake(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "OTBIntake"),
    OTBIntakeRollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Brake, false),
    OTBDropDownMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

OTBIntake::~OTBIntake()
{
}

void OTBIntake::resetState()
{
    state.dropDown = false;
    state.OTBisIntaking = false;
}

void OTBIntake::init()
{
    OTBIntakeRollerMotor.setConversion(1.0 / OTB_ROLLER_GEAR_RATIO * 360);
    OTBDropDownMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutBoolean("Intaking?", state.OTBisIntaking);
    table->PutBoolean("Dropdown?", state.dropDown);
}

void OTBIntake::assessInputs()
{
    if(operatorGamepad->GetXButtonPressed())
    {
        state.dropDown = true;
    }
    if(operatorGamepad->GetYButtonPressed())
    {
        state.OTBisIntaking = true;
    }
}

void OTBIntake::analyzeDashboard()
{
    table->PutBoolean("Intaking?", state.OTBisIntaking);
    table->PutBoolean("Dropdown?", state.dropDown);
}

void OTBIntake::assignOutputs()
{
    if(state.dropDown)
    {
        dropDown();
    }
    if(state.OTBisIntaking)
    {
        rollerIntake();
    }
}

void OTBIntake::dropDown()
{
    if(state.dropDown)
    {
        OTBDropDownMotor.setPosition(OTB_DROPDOWN_POSITION);
    }
    if(!state.dropDown)
    {
        OTBDropDownMotor.setPosition(OTB_INSIDE_POSITION);
    }
}

void OTBIntake::rollerIntake()
{
    OTBIntakeRollerMotor.setPower(OTBintakeRotMaxSpeed);
}

double OTBIntake::getOTBRollerSpeed()
{
    return OTBIntakeRollerMotor.getSpeed();
}

void OTBIntake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddBooleanProperty(
        "isDropdown",
        [this] {return state.dropDown;},
        nullptr
    );

    builder.AddBooleanProperty(
        "isIntaking",
        [this] {return state.OTBisIntaking;},
        nullptr
    );

    builder.AddDoubleProperty(
        "OTBrollerSpeed",
        [this] {return getOTBRollerSpeed();},
        nullptr
    );
}