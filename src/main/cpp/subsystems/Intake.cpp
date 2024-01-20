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
    state.activation = state.STOWED;
    state.intake = state.STAGNANT;
    state.detection = state.NOTE_NOTDETECTED;
}

void Intake::init()
{
    IntakeRollerMotor.setConversion(1.0 / OTB_ROLLER_GEAR_RATIO * 360);
    OTBDropDownMotor.setConversion(1.0 / OTB_DROPDOWN_GEAR_RATIO * 360);

    resetState();

    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assessInputs()
{
    if(operatorGamepad->GetXButtonPressed())
    {
        state.intake = state.INTAKING;
        state.activation = state.DEPOLOYED;
    }
    else
    {
        if(operatorGamepad->GetYButtonPressed())
        {
            state.intake = state.OUTTAKE;
            state.activation = state.DEPOLOYED;
        }
        else
        {
            state.intake = state.STAGNANT;
            state.activation = state.STOWED;
        }
    }
    if(operatorGamepad->GetAButtonPressed()) // placeholder for beam break sensor
    {
        state.detection = state.NOTE_DETECTED;
    }
    else
    {
        state.detection = state.NOTE_NOTDETECTED;
    }
}

void Intake::analyzeDashboard()
{
    table->PutNumber("Activation State", state.activation);
    table->PutNumber("Intake State", state.intake);
    table->PutNumber("Detection State", state.detection);
}

void Intake::assignOutputs()
{
}

void Intake::dropDown()
{
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

    /*builder.AddBooleanProperty(
        "isDropdown",
        [this] {return state.dropDown;},
        nullptr
    );

    builder.AddBooleanProperty(
        "isIntaking",
        [this] {return state.OTBisIntaking;},
        nullptr
    );*/

    builder.AddDoubleProperty(
        "OTBrollerSpeed",
        [this] {return getOTBRollerSpeed();},
        nullptr
    );
}