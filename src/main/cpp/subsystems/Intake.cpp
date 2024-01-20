#include "subsystems/Intake.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define OTB_ROLLER_GEAR_RATIO 4.0f
#define OTB_DROPDOWN_GEAR_RATIO 3.0f

#define OTB_DROPDOWN_POSITION 33.0f
#define OTB_INSIDE_POSITION 0.0f

Intake::Intake(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Intake"),
    IntakeRollerMotor(CANIDs::EXTERNAL_INTAKE, valor::NeutralMode::Brake, false),
    OTBDropDownMotor(CANIDs::EXTERNAL_DROPDOWN, valor::NeutralMode::Brake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{
}

void Intake::resetState()
{
    state.dropDown = false;
    state.OTBisIntaking = false;
}

void Intake::init()
{
    IntakeRollerMotor.setConversion(1.0 / OTB_ROLLER_GEAR_RATIO * 360);
    OTBDropDownMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutBoolean("Intaking?", state.OTBisIntaking);
    table->PutBoolean("Dropdown?", state.dropDown);
}

void Intake::assessInputs()
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

void Intake::analyzeDashboard()
{
    table->PutBoolean("Intaking?", state.OTBisIntaking);
    table->PutBoolean("Dropdown?", state.dropDown);
}

void Intake::assignOutputs()
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

void Intake::dropDown()
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

void Intake::rollerIntake()
{
    IntakeRollerMotor.setPower(IntakeRotMaxSpeed);
}

double Intake::getOTBRollerSpeed()
{
    return IntakeRollerMotor.getSpeed();
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
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