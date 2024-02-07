#include "subsystems/Shooter.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define PIVOT_ROTATE_K_VEL 90.0_rpm
#define PIVOT_ROTATE_K_ACC_MUL 0.5f
#define PIVOT_ROTATE_K_F 0.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_I 0.0f
#define PIVOT_ROTATE_K_D 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define LEFT_SHOOT_POWER 0.8f
#define LEFT_SHOOT_SPOOL 0.5f
#define LEFT_SHOOT_STANDBY 0.0f

#define RIGHT_SHOOT_POWER 0.8f
#define RIGHT_SHOOT_SPOOL 0.5f
#define RIGHT_SHOOT_STANDBY 0.0f

#define SHOOTER_ROTATE_GEAR_RATIO 1.0f
#define SHOOTER_ROTATE_FORWARD_LIMIT 90.0_deg
#define SHOOTER_ROTATE_REVERSE_LIMIT 0.0_deg

#define FLYWHEEL_MAX_RPM 5500.0f // theorhetical max is 5550
#define FLYWHEEL_DEFAULT_VELOCITY 0.0f
#define FLYWHEEL_VELOCITY_KP 0.6f
#define FLYWHEEL_VELOCITY_SPOOLING 200.0f
#define FLYWHEEL_VELOCITY_SHOOTING 2000.0f

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Shooter"),
    //pivotMotors(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    beamBreak(_beamBreak)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
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

    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);

    table->PutNumber("Left Shooter Power", LEFT_SHOOT_POWER);
    table->PutNumber("Left Shooter Spool", LEFT_SHOOT_SPOOL);
    table->PutNumber("Left Shooter Standby", LEFT_SHOOT_STANDBY);

    table->PutNumber("Right Shooter Power", RIGHT_SHOOT_POWER);
    table->PutNumber("Right Shooter Spool", RIGHT_SHOOT_SPOOL);
    table->PutNumber("Right Shooter Standby", RIGHT_SHOOT_STANDBY);

    table->PutNumber("Flywheel Stationary Velocity", FLYWHEEL_DEFAULT_VELOCITY);
    table->PutNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP);

    table->PutNumber("Flywheel Shooting Velocity", FLYWHEEL_VELOCITY_SHOOTING);
    table->PutNumber("Flywheel Spooling Velocity", FLYWHEEL_VELOCITY_SPOOLING);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr )
        return;
    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
        state.flywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(FLYWHEEL_VELOCITY_SHOOTING);
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
        state.flywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(FLYWHEEL_VELOCITY_SPOOLING);
    }
    else {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
        state.flywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
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

    state.leftShooterPower = table->GetNumber("Left Shooter Power", LEFT_SHOOT_POWER);
    state.leftSpoolPower = table->GetNumber("Left Shooter Spool", LEFT_SHOOT_SPOOL);
    state.leftStandbyPower = table->GetNumber("Left Shooter Standby", LEFT_SHOOT_STANDBY);

    state.rightShooterPower = table->GetNumber("Right Shooter Power", RIGHT_SHOOT_POWER);
    state.rightSpoolPower = table->GetNumber("Right Shooter Spool", RIGHT_SHOOT_SPOOL);
    state.rightStandbyPower = table->GetNumber("Right Shooter Standby", RIGHT_SHOOT_STANDBY);

    double flywheelTargetVelocity = table->GetNumber("Flywheel Stationary Velocity", state.flywheelTargetVelocity.to<double>());
    double kP = table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP);
}

void Shooter::assignOutputs()
{

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
    if (state.flywheelState == FLYWHEEL_STATE::SHOOTING) {
        leftFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getLeftFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
        rightFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getRightFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
    } else if (state.flywheelState == FLYWHEEL_STATE::SPOOLED) {
        leftFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getLeftFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
        rightFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getRightFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
    } else {
        leftFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getLeftFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
        rightFlywheelMotor.setPower(table->GetNumber("Flywheel Velocity P Value", FLYWHEEL_VELOCITY_KP)*(getRightFlywheelVelocityError().to<double>() / FLYWHEEL_MAX_RPM));
    }
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

    builder.AddDoubleProperty(
        "Left shoot power", 
        [this] { return state.leftShooterPower; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Right shoot power", 
        [this] { return state.rightShooterPower; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Left spooled power", 
        [this] { return state.leftSpoolPower; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Right spooled power", 
        [this] { return state.rightSpoolPower; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Target Flywheel Velocity",
        [this] {return state.flywheelTargetVelocity.to<double>();},
        nullptr
    );

}
