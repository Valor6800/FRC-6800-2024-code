#include "subsystems/Feeder.h"
#include <iostream>
#include <math.h>
#include <frc/AnalogOutput.h>
#include <frc/AnalogTrigger.h>
#include "frc/AnalogTriggerType.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"
#include "frc/AnalogTriggerOutput.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define INTAKE_FORWARD_POWER 0.8f
#define INTAKE_EXTRA_FORWARD_POWER 0.9f
#define INTAKE_REVERSE_POWER -1.0f

#define INTAKE_FORWARD_VELOCITY 24.0f
#define INTAKE_EXTRA_FORWARD_VELOCITY 27.0f

#define FEEDER_FORWARD_POWER 0.3f
#define FEEDER_INTAKE_POWER 0.1f
#define FEEDER_REVERSE_POWER -0.5f
#define FEEDER_UNJAM_POWER -0.2f

#define FEEDER_FORWARD_VELOCITY 9.0f
#define FEEDER_INTAKE_VELOCITY 3.0f

#define USE_UNJAM true

Feeder::Feeder(frc::TimedRobot *_robot, frc::AnalogTrigger* _feederBeamBreak, frc::AnalogTrigger* _feederBeamBreak2, valor::CANdleSensor* _leds, Shooter *_shooter) :
    valor::BaseSubsystem(_robot, "Feeder"),
    intakeMotor(CANIDs::INTERNAL_INTAKE, valor::NeutralMode::Coast, true),
    intakeBackMotor(CANIDs::INTERNAL_INTAKE_V2, valor::NeutralMode::Coast, true),
    feederMotor(CANIDs::FEEDER, valor::NeutralMode::Brake, true),
    feederBeamBreak(_feederBeamBreak),
    feederDebounceSensor(_robot, "FeederBanner"),
    stageBeamBreak(_feederBeamBreak2),
    stageDebounceSensor(_robot, "StageBanner"),
    leds(_leds),
    shooter(_shooter)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Shoot sequence-feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::INTAKE;
                }
            ),
            frc2::WaitCommand(1_s),
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = false;
                    state.feederState = Feeder::ROLLER_STATE::STAGNANT;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable full feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::SHOOT;
                    state.intakeState = Feeder::ROLLER_STATE::SHOOT;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Disable full feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::STAGNANT;
                    state.intakeState = Feeder::ROLLER_STATE::STAGNANT;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::SHOOT;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Disable feeder", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.feederState = Feeder::ROLLER_STATE::STAGNANT;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable intake", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.intakeState = Feeder::ROLLER_STATE::INTAKE;
                    state.feederState = Feeder::ROLLER_STATE::INTAKE;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable only intake", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.intakeState = Feeder::ROLLER_STATE::INTAKE;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Disable intake", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.intakeState = Feeder::ROLLER_STATE::STAGNANT;
                }
            )
        )
    ).ToPtr());
}

void Feeder::resetState()
{
    state.intakeState = STAGNANT;
    state.feederState = STAGNANT;
}

void Feeder::init()
{
    resetState();

    valor::PIDF intakePID = valor::PIDF();
    intakePID.P = 0.0006;
    intakePID.maxVelocity = 70;
    intakePID.maxAcceleration = 200;
    intakeMotor.setPIDF(intakePID, 0);
    intakeMotor.setConversion(1.0, 1.0);
    intakeBackMotor.setPIDF(intakePID, 0);
    intakeBackMotor.setConversion(1.0, 1.0);

    intakeMotor.setMaxCurrent(80);
    intakeMotor.setVoltageCompensation(10);

    intakeBackMotor.setMaxCurrent(60);
    intakeBackMotor.setVoltageCompensation(10);

    feederMotor.setVoltageCompensation(10);
    valor::PIDF feederPID = valor::PIDF();
    feederPID.P = 0.0006;
    feederPID.maxVelocity = 32;
    feederPID.maxAcceleration = 100;
    feederMotor.setPIDF(feederPID, 0);
    feederMotor.setConversion(1.0, 1.0);

    feederDebounceSensor.setGetter([this] { return (!feederBeamBreak->GetInWindow()); });
    feederDebounceSensor.setRisingEdgeCallback([this] {
        state.feedTrip = true;
        feederMotor.setPower(0);
        intakeMotor.setPower(0);
        intakeBackMotor.setPower(0);
        // leds->setAnimation(0, valor::CANdleSensor::AnimationType::Rainbow, valor::CANdleSensor::RGBColor(255,0,0));
    });
    stageDebounceSensor.setGetter([this] { return (!stageBeamBreak->GetInWindow()); });
    stageDebounceSensor.setRisingEdgeCallback([this] {
        state.stageTrip = true;
        feederMotor.setPower(FEEDER_INTAKE_POWER);
        intakeMotor.setPower(INTAKE_FORWARD_POWER * .5);
        intakeBackMotor.setPower(INTAKE_FORWARD_POWER * .5);
        driverGamepad->setRumble(true);
    });

    // intakeDebounceSensor.setGetter([this] { return !intakeBeamBreak->GetInWindow(); });
    // intakeDebounceSensor.setRisingEdgeCallback([this] {
    //     driverGamepad->setRumble(true);
    //     if (true) {
    //         intakeMotor.setPower(INTAKE_FORWARD_POWER * .75);
    //         intakeBackMotor.setPower(INTAKE_REVERSE_POWER * .75);
    //     }
    // });

    table->PutNumber("Intake tuning speed", INTAKE_FORWARD_POWER);
}

void Feeder::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;
    if (driverGamepad->rightTriggerActive()) {
        state.intakeState = ROLLER_STATE::SHOOT;
        if (driverGamepad->GetBButton())
            state.feederState = ROLLER_STATE::OUTTAKE;
        else
            state.feederState = ROLLER_STATE::SHOOT;
    } else if (driverGamepad->GetRightBumper()) {
        state.intakeState = ROLLER_STATE::INTAKE;
        state.feederState = ROLLER_STATE::INTAKE;
    } else if(driverGamepad->GetLeftBumper()) {
        state.intakeState = ROLLER_STATE::OUTTAKE;
        state.feederState = ROLLER_STATE::OUTTAKE;
    } else {
        state.intakeState = ROLLER_STATE::STAGNANT;
        state.feederState = ROLLER_STATE::STAGNANT;
    }
}

void Feeder::analyzeDashboard()
{
    state.tuningPower = table->PutNumber("Intake tuning speed", INTAKE_FORWARD_POWER);
    if (table->GetBoolean("Tuning", false)) {
        state.intakeState = ROLLER_STATE::TUNING;
    }
    state.stageTrip = !stageBeamBreak->GetInWindow();
    state.feedTrip = !feederBeamBreak->GetInWindow();
    

    if (state.feederState == ROLLER_STATE::SHOOT || state.feederState == ROLLER_STATE::OUTTAKE) {
        driverGamepad->setRumble(false);
    }
    if (!feederBeamBreak->GetInWindow()) {
        driverGamepad->setRumble(true);
        // leds->setColor(0, valor::CANdleSensor::LIGHT_BLUE);
    } else {
        driverGamepad->setRumble(false);
        // leds->setColor(0, valor::CANdleSensor::RED);
    }
    if (driverGamepad != nullptr && driverGamepad->IsConnected() && !(frc::DriverStation::IsTeleop() && frc::DriverStation::IsEnabled())) {
        driverGamepad->setRumble(false);
    }
}

void Feeder::assignOutputs()
{
    if(state.intakeState == ROLLER_STATE::SHOOT) {
        intakeMotor.setSpeed(INTAKE_FORWARD_VELOCITY);
        intakeBackMotor.setSpeed(INTAKE_FORWARD_VELOCITY);
    } else if(state.intakeState == ROLLER_STATE::INTAKE) {
        if (state.stageTrip) {
            intakeMotor.setSpeed(INTAKE_FORWARD_VELOCITY * .5);
            intakeBackMotor.setSpeed(INTAKE_FORWARD_VELOCITY * .5);
        } else if (state.feedTrip) {
            intakeMotor.setPower(0);
            intakeBackMotor.setPower(0);
        } else {
            if (shooter->state.pivotLowered) {
                intakeMotor.setSpeed(INTAKE_EXTRA_FORWARD_VELOCITY);
                intakeBackMotor.setSpeed(INTAKE_FORWARD_VELOCITY);
            }
        }
    } else if(state.intakeState == ROLLER_STATE::OUTTAKE) {
        intakeMotor.setPower(INTAKE_REVERSE_POWER);
        intakeBackMotor.setPower(INTAKE_REVERSE_POWER);
    } else if(state.intakeState == ROLLER_STATE::TUNING) {
        intakeMotor.setPower(state.tuningPower);
        intakeBackMotor.setPower(state.tuningPower);
    } else {
        intakeMotor.setSpeed(0);
        intakeBackMotor.setSpeed(0);
    }
    
    if (state.feederState == ROLLER_STATE::SHOOT) {
        feederMotor.setSpeed(FEEDER_FORWARD_VELOCITY);
    } else if(state.feederState == ROLLER_STATE::INTAKE) {
        if (state.feedTrip) {
            feederMotor.setSpeed(0);
        } else if (state.stageTrip) {
            feederMotor.setSpeed(FEEDER_INTAKE_VELOCITY);
        } else {
            feederMotor.setSpeed(FEEDER_FORWARD_VELOCITY);
        }
    } else if(state.feederState == ROLLER_STATE::OUTTAKE) {
        feederMotor.setPower(FEEDER_REVERSE_POWER);
    } else {
        feederMotor.setPower(0);
    }
}

void Feeder::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "intakeState",
        [this] {return state.intakeState;},
        nullptr
    );

    builder.AddDoubleProperty(
        "feederState",
        [this] {return state.feederState;},
        nullptr
    );

    builder.AddBooleanProperty(
        "Stage Trip",
        [this]{return state.stageTrip;},
        nullptr
    );

    builder.AddBooleanProperty(
        "Feed Trip",
        [this]{return state.feedTrip;},
        nullptr
    );
    //
    // builder.AddBooleanProperty(
    //     "Unjam",
    //     [this]{return state.unjam;},
    //     nullptr
    // );
}
