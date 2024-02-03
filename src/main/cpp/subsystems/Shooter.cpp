#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define PIVOT_ROTATE_K_VEL 90.0_rpm
#define PIVOT_ROTATE_K_ACC_MUL 0.5f
#define PIVOT_ROTATE_K_F 0.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_I 0.0f
#define PIVOT_ROTATE_K_D 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define FLYWHEEL_ROTATE_K_F 0.0115f // v/rpm
#define FLYWHEEL_ROTATE_K_P 0.00345f // v/rpm

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define FLYWHEEL_MAX_RPM 4500 // theorhetical max is 5550

#define SHOOT_POWER FLYWHEEL_MAX_RPM * 1.0_rpm
#define SPOOL_POWER FLYWHEEL_MAX_RPM * 0.75_rpm
#define STANDBY_POWER 0.0_rpm

#define FLYWHEEL_ROTATE_GEAR_RATIO 1.0f
#define FLYWHEEL_ROTATE_FORWARD_LIMIT 90.0_deg
#define FLYWHEEL_ROTATE_REVERSE_LIMIT 0.0_deg

#define FLYWHEEL_DEFAULT_VELOCITY 0.0f
#define FLYWHEEL_VELOCITY_KP 0.6f
#define FLYWHEEL_VELOCITY_SPOOLING 200.0f
#define FLYWHEEL_VELOCITY_SHOOTING 2000.0f

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Shooter"),
    //pivotMotors(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
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
    state.flywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
}

void Shooter::init()
{
    pivotPID.velocity = PIVOT_ROTATE_K_VEL.to<double>();
    pivotPID.acceleration = PIVOT_ROTATE_K_ACC_MUL;
    pivotPID.F = PIVOT_ROTATE_K_F;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.I = PIVOT_ROTATE_K_I;
    pivotPID.D = PIVOT_ROTATE_K_D;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.aFFTarget = PIVOT_ROTATE_K_AFF_POS;

    valor::PIDF flywheelPID;
    flywheelPID.F = FLYWHEEL_ROTATE_K_F;
    flywheelPID.P = FLYWHEEL_ROTATE_K_P;
    
    leftFlywheelMotor.setupFollower(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, false);
    leftFlywheelMotor.setPIDF(flywheelPID, 0);
    leftFlywheelMotor.setReverseLimit(0.0);
    leftFlywheelMotor.setConversion(1);
    
    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);

    table->PutNumber("Flywheel Shoot RPM", SHOOT_POWER.to<double>());
    table->PutNumber("Flywheel Spool RPM", SPOOL_POWER.to<double>());
    table->PutNumber("Flywheel Standby RPM", STANDBY_POWER.to<double>());

    table->PutNumber("Flywheel Max RPM", FLYWHEEL_MAX_RPM);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr )
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
            state.flywheelTargetVelocity = units::revolutions_per_minute_t(table->GetNumber("Flywheel Shoot RPM", SHOOT_POWER.to<double>()));
            break;

        case NOT_SHOOTING:
            state.flywheelTargetVelocity = units::revolutions_per_minute_t(table->GetNumber("Flywheel Standby RPM", STANDBY_POWER.to<double>()));
            break;

        default:
            state.flywheelTargetVelocity = units::revolutions_per_minute_t(table->GetNumber("Flywheel Spool RPM", SPOOL_POWER.to<double>()));
            break;
    }
}

void Shooter::assignOutputs()
{
    leftFlywheelMotor.setSpeed(state.flywheelTargetVelocity.to<double>());
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

    /* builder.AddIntegerProperty(
        "pivot state",
        [this] {return state.pivot;},
        nullptr
    ); */

    builder.AddDoubleProperty("FlyWheel current velocity", [this] {return leftFlywheelMotor.getSpeed();}, nullptr);

    builder.AddDoubleProperty(
        "Flywheel target velocity",
        [this] {return state.flywheelTargetVelocity.to<double>();},
        nullptr
    );
}
