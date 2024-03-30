#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc/PWM.h>

#define FORWARD_LIMIT 0.0f
#define REVERSE_LIMIT 0.0f
#define EXTENDED_POS 0.0f
#define RETRACTED_POS 0.0f
#define RESTING_POS 0.0f

#define MAX_SPEED 0.0f

#define LATCH_PULSE_TIME 1.6_ms
#define UNLATCH_PULSE_TIME 2.5_ms

#define LATCH_POS 0.55f
#define UNLATCH_POS 1.0f

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(CANIDs::CLIMBER_LEAD, valor::NeutralMode::Brake, true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Climber::~Climber()
{

}

void Climber::init()
{
    climbMotors.setupFollower(CANIDs::CLIMBER_FOLLOW, true);
    climbMotors.setForwardLimit(FORWARD_LIMIT);
    climbMotors.setReverseLimit(REVERSE_LIMIT);

    climbUp = std::move(UpClimb().ToPtr());
    climbDown = std::move(DownClimb().ToPtr());

    servo = new frc::PWM(9, true);
    servo->SetBounds(2000_us, 3_us, 1000_us, 3_us, 500_us);
}

void Climber::resetState()
{

}

void Climber::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

    if (driverGamepad->DPadDown()) {
        if (climbDown.IsScheduled()){
            climbDown.Cancel();
        } else if (!climbDown.IsScheduled()){
            climbDown.Schedule();
        }
    }
    if (driverGamepad->DPadUp()){
        if (climbUp.IsScheduled()){
        climbUp.Cancel();
        } else if(!climbUp.IsScheduled()){
        climbUp.Schedule();
        }
    }
    if (operatorGamepad->rightStickYActive() || operatorGamepad->DPadUp() || operatorGamepad->DPadDown()){
        if (climbUp.IsScheduled()){
            climbUp.Cancel();
        } else if (climbDown.IsScheduled()){
            climbDown.Cancel();
        }
    }

    if (operatorGamepad->DPadUp()){
        state.latchState = UNLATCH;
    } else if (operatorGamepad->DPadDown()){
        state.latchState = LATCH;
    }
    if (operatorGamepad->rightStickYActive()){
        state.climbState = MANUAL;
    }    
}

void Climber::analyzeDashboard()
{

}

void Climber::assignOutputs()
{
    if (!drive->state.pitMode){
        if (state.climbState == EXTEND){
            servo->SetPulseTime(LATCH_PULSE_TIME); //unlatch
                if (servo->GetPosition() == LATCH_POS){
                    climbMotors.setPosition(EXTENDED_POS);
                }
        } else if(state.climbState == RETRACT){
            servo->SetPulseTime(UNLATCH_PULSE_TIME); //re-latch
                if (servo->GetPosition() == UNLATCH_POS){
                    climbMotors.setPosition(RETRACTED_POS);
                }
        } else if(state.climbState == DISABLE){
            climbMotors.setPosition(RESTING_POS);
        } else {
            climbMotors.setPosition(RESTING_POS);
        }

        if (state.latchState == UNLATCH){
            servo->SetPulseTime(LATCH_PULSE_TIME);
        } else if(state.latchState == LATCH){
            servo->SetPulseTime(UNLATCH_PULSE_TIME);
        }

        if (state.climbState == MANUAL && (servo->GetPosition() == LATCH_POS || servo->GetPosition() == UNLATCH_POS)){
            climbMotors.setPosition(MAX_SPEED * operatorGamepad->rightStickY(2));
        }
    }
}

frc2::SequentialCommandGroup Climber::UpClimb()
{
    return frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.climbState = EXTEND;
                }
            )
    );
}

frc2::SequentialCommandGroup Climber::DownClimb()
{
    return frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.climbState = RETRACT;
                }
            )
    );
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Servo Position",
        [this] {return servo->GetPosition();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Servo Pulse Time",
        [this] {return servo->GetPulseTime().to<double>();},
        nullptr
    );
    builder.AddIntegerProperty(
        "Climber State",
        [this] {return state.climbState;},
        nullptr
    );
    builder.AddIntegerProperty(
        "Latch State",
        [this] {return state.latchState;},
        nullptr
    );
}

