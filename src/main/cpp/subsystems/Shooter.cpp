#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define PIVOT_ROTATE_K_VEL 90.0f
#define PIVOT_ROTATE_K_ACC 500.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define PIVOT_GEAR_RATIO 1.0f
#define PIVOT_REVERSE_LIMIT 0.0f
#define PIVOT_FORWARD_LIMIT 90.0f

#define FLYWHEEL_ROTATE_K_VEL 4500.0f
#define FLYWHEEL_ROTATE_K_ACC 4500.0f
#define FLYWHEEL_ROTATE_K_P 0.0f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define SHOOT_POWER 4000.0f
#define SPOOL_POWER 3000.0f
#define STANDBY_POWER 0.0f

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Shooter"),
    //pivotMotors(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    beamBreak(_beamBreak)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Shoot sequence-shooter", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.flywheelState = Shooter::FLYWHEEL_STATE::SHOOTING;
                }
            ),
            frc2::WaitCommand(1_s),
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = false;
                    state.flywheelState = Shooter::FLYWHEEL_STATE::NOT_SHOOTING;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Spool", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.flywheelState = Shooter::FLYWHEEL_STATE::SPOOLED;
                }
            )
        )
    ).ToPtr());
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::SUBWOOFER;
    state.leftFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
    state.rightFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
}

void Shooter::init()
{
    valor::PIDF pivotPID;
    pivotPID.maxVelocity = PIVOT_ROTATE_K_VEL;
    pivotPID.maxAcceleration = PIVOT_ROTATE_K_ACC;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.aFFTarget = PIVOT_ROTATE_K_AFF_POS;

    valor::PIDF flywheelPID;
    flywheelPID.maxVelocity = FLYWHEEL_ROTATE_K_VEL;
    flywheelPID.maxAcceleration = FLYWHEEL_ROTATE_K_ACC;
    flywheelPID.P = FLYWHEEL_ROTATE_K_P;
    
    leftFlywheelMotor.setPIDF(flywheelPID, 0);
    rightFlywheelMotor.setPIDF(flywheelPID, 0);
    
    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);

    table->PutNumber("Left Flywheel Shoot RPM", SHOOT_POWER);
    table->PutNumber("Left Flywheel Spool RPM", SPOOL_POWER);
    table->PutNumber("Left Flywheel Standby RPM", STANDBY_POWER);

    table->PutNumber("Right Flywheel Shoot RPM", SHOOT_POWER);
    table->PutNumber("Right Flywheel Spool RPM", SPOOL_POWER);
    table->PutNumber("Right Flywheel Standby RPM", STANDBY_POWER);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;

    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    }
    else {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } 

    //PIVOT LOGIC
    // if (driverGamepad->GetAButton()) {
    //     state.pivot = PivotState::SUBWOOFER;
    // }
   /* else if (operatorGamepad->GetRightBumperPressed()) {
        state.pivot = PivotState::PODIUM;
    }
    else if (operatorGamepad->GetLeftBumperPressed()) { 
        state.pivot = PivotState::STARTING_LINE;
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.pivot = PivotState::TRACKING;*/
    
}

void Shooter::analyzeDashboard()
{
    state.pivotAngle = 0.0_deg;

    //NEED PIVOT MOTOR
    // if(state.pivot == PivotState::SUBWOOFER){
    //     pivotMotors.setPosition(SUBWOOFER_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::PODIUM){
    //     pivotMotors.setPosition(PODIUM_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::STARTING_LINE){
    //     pivotMotors.setPosition(STARTING_LINE_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::TRACKING){
    //     pivotMotors.setPosition(calculatingPivotingAngle.to<double>());
    // }

    //SHOOTER
    switch (state.flywheelState) {

        case SHOOTING:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Shoot RPM", SHOOT_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Shoot RPM", SHOOT_POWER));
            break;

        case NOT_SHOOTING:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Standby RPM", STANDBY_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Standby RPM", STANDBY_POWER));
            break;

        default:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Spool RPM", SPOOL_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Spool RPM", SPOOL_POWER));
            break;
    }
}

void Shooter::assignOutputs()
{
    leftFlywheelMotor.setSpeed(state.leftFlywheelTargetVelocity.to<double>());
    rightFlywheelMotor.setSpeed(state.rightFlywheelTargetVelocity.to<double>());
}

units::degree_t Shooter::calculatePivotAngle(){
    units::degree_t targetPivotAngle = units::degree_t(3);
    return targetPivotAngle;
}

void Shooter::calculateRootsT(){
    // add future code for solving roots of the quartic that results from the vector expression
}

void Shooter::bisectionTheorem(){
    // neccessary for calculateRootsT where the bisection method is used to estimate these values
}

void Shooter::InitSendable(wpi::SendableBuilder& builder){

    builder.SetSmartDashboardType("Shooter");

    builder.AddIntegerProperty(
        "flywheel state",
        [this] {return state.flywheelState;},
        nullptr
    );

    builder.AddIntegerProperty(
        "pivot state",
        [this] {return state.pivotState;},
        nullptr
    );

    builder.AddIntegerProperty(
        "pivot target angle",
        [this] {return state.pivotAngle.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Left Flywheel target velocity",
        [this] {return state.leftFlywheelTargetVelocity.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Left Flywheel target velocity",
        [this] {return state.rightFlywheelTargetVelocity.to<double>();},
        nullptr
    );
}
