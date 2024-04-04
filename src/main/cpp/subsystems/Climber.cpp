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

#define FORWARD_LIMIT 16.5f
#define REVERSE_LIMIT 1.0f
#define EXTENDED_POS 0.0f
#define RETRACTED_POS 0.0f
#define RESTING_POS 0.0f

#define CLIMB_MAX_SPEED 0.0f
#define CLIMB_MAX_ACCEL 0.0f
#define CLIMB_K_P 0.0f
#define CLIMB_K_ERROR 0.00f
#define CLIMB_K_AFF 0.00f
#define CONVERSION 14.72f

#define UNLATCH_PULSE_TIME 2500_us
#define LATCH_PULSE_TIME 1900_us

#define LATCH_POS 0.55f
#define UNLATCH_POS 1.0f

Climber::Climber(frc::TimedRobot *_robot, valor::CANdleSensor *_leds) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(nullptr),
    leds(_leds)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    table->PutBoolean("Climber overriding leds", false);
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = DISABLE;
    state.latchState = LATCH;
}

void Climber::init()
{
    valor::PIDF climbPID;
    climbPID.maxVelocity = CLIMB_MAX_SPEED;
    climbPID.maxAcceleration = CLIMB_MAX_ACCEL;
    climbPID.P = CLIMB_K_P;
    climbPID.error = CLIMB_K_ERROR;
    climbPID.aFF = CLIMB_K_AFF;

    climbPID.aFFType = valor::FeedForwardType::LINEAR;
    bool climberI = false;
    climbMotors = new valor::PhoenixController(
        CANIDs::CLIMBER_LEAD,
        valor::NeutralMode::Brake,
        climberI,
        CONVERSION,
        1.225 * M_PI,
        climbPID,
        12.0,
        true,
        "baseCAN"
    );
    climbMotors->setupFollower(CANIDs::CLIMBER_FOLLOW, climberI);
    climbMotors->setRange(0, REVERSE_LIMIT, FORWARD_LIMIT);
    climbMotors->enableFOC(true);

    servo = new frc::PWM(0, true);
    servo->SetBounds(2000_us, 3_us, 1000_us, 3_us, 500_us);
    table->PutNumber("Servo Pos", 500);
    table->PutBoolean("Climber tuning", false);
    state.climbState = DISABLE;
    state.latchState = LATCH;

    resetState();
}

void Climber::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;

    if (driverGamepad->DPadUp()) {
        state.climbState = EXTEND;
    } else if (driverGamepad->DPadDown()){
        state.climbState = RETRACT;
    }

    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

    if (operatorGamepad->DPadUp()){
        state.latchState = UNLATCH;
    } else if (operatorGamepad->DPadDown()){
        state.latchState = LATCH;
    }
    if (operatorGamepad->rightStickYActive()){
        state.climbState = MANUAL;
        leds->setAnimation(valor::CANdleSensor::AnimationType::Rainbow, valor::CANdleSensor::RGBColor{0, 0, 0});
    } else {
        leds->clearAnimation();
    }
}

void Climber::analyzeDashboard()
{
    if (climbMotors->getPosition() > 1.0) {
        table->PutBoolean("Climber overriding leds", true);
        leds->setColor(0, valor::CANdleSensor::VALOR_PURPLE);
        leds->setColor(1, valor::CANdleSensor::VALOR_PURPLE);
    } else {
        table->PutBoolean("Climber overriding leds", false);
    }
}

void Climber::assignOutputs()
{
    /*if (!drive->state.pitMode){
        if (state.climbState == CLIMB_STATE::EXTEND){
            servo->SetPulseTime(LATCH_PULSE_TIME); //unlatch
                if (inPosition()){
                    climbMotors->setPosition(EXTENDED_POS);
                }
        } else if(state.climbState == CLIMB_STATE::RETRACT){
            servo->SetPulseTime(UNLATCH_PULSE_TIME); //re-latch
                if (inPosition()){
                    climbMotors->setPosition(RETRACTED_POS);
                }
        } else if(state.climbState == CLIMB_STATE::DISABLE){
            climbMotors->setPosition(RESTING_POS);
        } else {
            climbMotors->setPosition(RESTING_POS);
        }
    }*/

    // TODO: Set position with state.targetPos when valor::Servo gets implemented
    if (state.latchState == LATCH_STATE::UNLATCH){
        servo->SetPulseTime(UNLATCH_PULSE_TIME);
    } else if(state.latchState == LATCH_STATE::LATCH){
        servo->SetPulseTime(LATCH_PULSE_TIME);
    }

    if (state.climbState == MANUAL){
        double opIn = operatorGamepad->rightStickY(2);
        if (state.latchState == UNLATCH || opIn < 0) {
            climbMotors->setPower(opIn);
        }
    }
    if (table->GetBoolean("Climber tuning", false)) { 
        double st = table->GetNumber("Servo Pos", 500) / 1000000;
        if (servo != nullptr)
            servo->SetPulseTime(units::second_t{st}); // Can't cast straight to microseconds
    }
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

