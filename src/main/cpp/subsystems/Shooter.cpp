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

#define SHOOTER_K_P 0.00007f
#define SHOOTER_K_F 0.00014f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define LEFT_SHOOT_POWER 4500.0f
#define LEFT_SHOOT_SPOOL 3500.0f
#define LEFT_SHOOT_STANDBY 0.0f

#define SHOOTER_ROTATE_GEAR_RATIO 1.0f
#define SHOOTER_ROTATE_FORWARD_LIMIT 90.0_deg
#define SHOOTER_ROTATE_REVERSE_LIMIT 0.0_deg

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Shooter"),
    //pivotMotors(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    beamBreak(_beamBreak)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::SUBWOOFER;
}

void Shooter::init()
{
    valor::PIDF pivotPID;
    pivotPID.velocity = PIVOT_ROTATE_K_VEL.to<double>();
    pivotPID.acceleration = PIVOT_ROTATE_K_ACC_MUL;
    pivotPID.F = PIVOT_ROTATE_K_F;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.I = PIVOT_ROTATE_K_I;
    pivotPID.D = PIVOT_ROTATE_K_D;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.aFFTarget = PIVOT_ROTATE_K_AFF_POS;

    valor::PIDF shooterPIDF;
    shooterPIDF.P = SHOOTER_K_P;
    shooterPIDF.F = SHOOTER_K_F;

    leftFlywheelMotor.setupFollower(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, true);
    leftFlywheelMotor.setReverseLimit(0);
    leftFlywheelMotor.setPIDF(shooterPIDF, 0);
    leftFlywheelMotor.setConversion(1.0);

    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);

    table->PutNumber("Shooter RPM", LEFT_SHOOT_POWER);
    table->PutNumber("Shooter Spool RPM", LEFT_SHOOT_SPOOL);
    table->PutNumber("Shooter Standby RPM", LEFT_SHOOT_STANDBY);    

    resetState();

}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr )
        return;
    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    }
    else {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
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

    state.leftShooterPower = table->GetNumber("Shooter RPM", LEFT_SHOOT_POWER);
    state.leftSpoolPower = table->GetNumber("Shooter Spool RPM", LEFT_SHOOT_SPOOL);
    state.leftStandbyPower = table->GetNumber("Shooter Standby RPM", LEFT_SHOOT_STANDBY);
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
        leftFlywheelMotor.setSpeed(state.leftShooterPower);
    } else if (state.flywheelState == FLYWHEEL_STATE::SPOOLED) {
        leftFlywheelMotor.setSpeed(state.leftSpoolPower);
    } else {
        leftFlywheelMotor.setSpeed(state.leftStandbyPower);
    }
}

units::degree_t calculatePivotAngle(){
    units::degree_t targetPivotAngle = units::degree_t(3);
    return targetPivotAngle;
}

void calculateRootsT(){
    // add future code for solving roots of the quartic that results from the vector expression
}

void bisectionTheorem(){
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
}