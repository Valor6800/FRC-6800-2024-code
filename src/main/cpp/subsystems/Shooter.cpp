#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#define PIVOT_ROTATE_K_VEL 90.0f
#define PIVOT_ROTATE_K_ACC 500.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define PIVOT_GEAR_RATIO 1.0f
#define PIVOT_REVERSE_LIMIT 0.0f
#define PIVOT_FORWARD_LIMIT 90.0f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.000005f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define xING_LINE_ANG 60.0_deg

#define LEFT_SHOOT_POWER 66.33f
#define LEFT_SPOOL_POWER 50.0f
#define LEFT_STANDBY_POWER 0.0f

#define RIGHT_SHOOT_POWER 66.33f
#define RIGHT_SPOOL_POWER 50.0f
#define RIGHT_STANDBY_POWER 0.0f

#define FLYWHEEL_ROTATE_GEAR_RATIO 1.0f
#define FLYWHEEL_ROTATE_FORWARD_LIMIT 90.0_deg
#define FLYWHEEL_ROTATE_REVERSE_LIMIT 0.0_deg

#define FLYWHEEL_DEFAULT_VELOCITY 0.0f
#define FLYWHEEL_VELOCITY_KP 0.6f
#define FLYWHEEL_VELOCITY_SPOOLING 200.0f
#define FLYWHEEL_VELOCITY_SHOOTING 2000.0f

#define SPEAKER_Y 5.543042_m
#define SPEAKER_BLUE_X 0.0_m
#define SPEAKER_RED_X 16.4846_m
#define SPEAKER_X_OFFSET 0.15f
#define SPEAKER_Y_OFFSET 0.00f
#define SPEAKER_HEIGHT 2.0431125f
#define SPEAKER_Z_OFFSET 0.0f

#define PROJECTILE_SPEED 40.0f
#define GRAVITY 9.81f
#define SHOOTER_Z_OFFSET 0.0f

#define PIVOT_SUBWOOFER_POSITION 0.0f
#define PIVOT_PODIUM_POSITION 0.00f
#define PIVOT_STARTING_LINE_POSITION 0.00f

#define PIVOT_ROTATE_GEAR_RATIO 0.0
#define PIVOT_ROTATE_FORWARD_LIMIT 0.0
#define PIVOT_ROTATE_REVERSE_LIMIT 0.0

#define ARBITRARY_LARGE_VALUE 10000000.0f
#define EPSILON 0.00001f

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak, Drivetrain *_drive) :
    valor::BaseSubsystem(_robot, "Shooter"),
    pivotMotor(CANIDs::PIVOT, valor::NeutralMode::Brake, false),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    beamBreak(_beamBreak),
    drivetrain(_drive)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::SUBWOOFER;
    state.leftFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
    state.rightFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
    state.targetPivotAngle = 0_rad;
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
    
    leftFlywheelMotor.setConversion(1);
    leftFlywheelMotor.setPIDF(flywheelPID, 0);
    rightFlywheelMotor.setConversion(1);
    rightFlywheelMotor.setPIDF(flywheelPID, 0);
    
    pivotMotor.setConversion(1.0 / PIVOT_ROTATE_GEAR_RATIO * 360);
    pivotMotor.setForwardLimit(PIVOT_ROTATE_FORWARD_LIMIT);
    pivotMotor.setReverseLimit(PIVOT_ROTATE_REVERSE_LIMIT);
    pivotMotor.setPIDF(pivotPID, 0);

    table->PutNumber("Left Flywheel Shoot RPM", LEFT_SHOOT_POWER);
    table->PutNumber("Left Flywheel Spool RPM", LEFT_SPOOL_POWER);
    table->PutNumber("Left Flywheel Standby RPM", LEFT_STANDBY_POWER);

    table->PutNumber("Right Flywheel Shoot RPM", RIGHT_SHOOT_POWER);
    table->PutNumber("Right Flywheel Spool RPM", RIGHT_SPOOL_POWER);
    table->PutNumber("Right Flywheel Standby RPM", RIGHT_STANDBY_POWER);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    }
    else {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } 

    if (operatorGamepad->GetRightBumperPressed()) {
        state.pivot = PIVOT_STATE::PODIUM;
    }
    else if (operatorGamepad->GetLeftBumperPressed()) { 
        state.pivot = PIVOT_STATE::STARTING_LINE;
    }
    else if (operatorGamepad->GetAButton()) {
        state.pivot = PIVOT_STATE::TRACKING;    
    }
    else if (operatorGamepad->GetBButton()){
        state.pivot = PIVOT_STATE::SUBWOOFER;
    }
}

void Shooter::analyzeDashboard()
{
    switch(state.pivot){
        case PODIUM:
            state.targetPivotAngle = units::radian_t(PIVOT_PODIUM_POSITION);
            break;
        
        case STARTING_LINE:
            state.targetPivotAngle = units::radian_t(PIVOT_STARTING_LINE_POSITION);
            break;
        
        case TRACKING:
            getTargetPivotAngle(true);
            break;

        default:
            getTargetPivotAngle(true); 
            break;
    }

    switch (state.flywheelState) {

        case SHOOTING:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Shoot RPM", LEFT_SHOOT_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Shoot RPM", RIGHT_SHOOT_POWER));
            break;

        case NOT_SHOOTING:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Standby RPM", LEFT_STANDBY_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Standby RPM", RIGHT_STANDBY_POWER));
            break;

        default:
            state.leftFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Left Flywheel Spool RPM", LEFT_SPOOL_POWER));
            state.rightFlywheelTargetVelocity = units::revolutions_per_minute_t(
                table->GetNumber("Right Flywheel Spool RPM", RIGHT_SPOOL_POWER));
            break;
    }

    double changeInY = drivetrain->getCalculatedPose_m().Y().to<double>() - SPEAKER_Y.to<double>();

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        double changeInX = drivetrain->getCalculatedPose_m().X().to<double>() - SPEAKER_RED_X.to<double>();
        state.distanceFromSpeaker = units::meter_t{sqrtf(pow(changeInX, 2) + pow(changeInY, 2))};
    }
    else{
        double changeInX = drivetrain->getCalculatedPose_m().X().to<double>() - SPEAKER_BLUE_X.to<double>();
        state.distanceFromSpeaker = units::meter_t{sqrtf(pow(changeInX, 2) + pow(changeInY, 2))};
    }
}

void Shooter::assignOutputs()
{
    // leftFlywheelMotor.setSpeed(state.flywheelTargetVelocity.to<double>());
    pivotMotor.setPosition(state.targetPivotAngle.to<double>());
}

// both gravity and laser work
void Shooter::getTargetPivotAngle(bool laser){
    units::meter_t robotX = drivetrain->getCalculatedPose_m().X();
    units::meter_t robotY = drivetrain->getCalculatedPose_m().Y();
    double distance = 0.0;
    double changeInY = robotY.to<double>() - SPEAKER_Y.to<double>();
    if(laser){
        if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
        double changeInX = robotX.to<double>() - SPEAKER_BLUE_X.to<double>();
        distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
        }
        else{
            double changeInX = robotX.to<double>() - SPEAKER_RED_X.to<double>();
            distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
        }
        double angle = atan2(SPEAKER_HEIGHT + table->GetNumber("Speaker Z Offset", SPEAKER_Z_OFFSET), state.distanceFromSpeaker.to<double>());
        state.targetPivotAngle = units::radian_t(angle);
    }
    else{
        if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
        double changeInX = robotX.to<double>() - SPEAKER_BLUE_X.to<double>();
        distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
    }
        else{
            double changeInX = robotX.to<double>() - SPEAKER_RED_X.to<double>();
            distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
        }
        double angle = atan2(pow(PROJECTILE_SPEED, 2) - sqrt(pow(PROJECTILE_SPEED, 4) - (GRAVITY*(GRAVITY*pow(state.distanceFromSpeaker.to<double>(), 2) + 2*SPEAKER_HEIGHT*pow(PROJECTILE_SPEED, 2)))), GRAVITY*state.distanceFromSpeaker.to<double>());
        state.targetPivotAngle = units::radian_t(angle);
    }
}

void Shooter::getArcTargetPivotAngle(){
    units::meter_t robotX = drivetrain->getPose_m().X();
    units::meter_t robotY = drivetrain->getPose_m().Y();
    double distance = sqrt(pow(robotX.to<double>(), 2) + pow(robotY.to<double>(), 2));
    double changeInY = robotY.to<double>() - SPEAKER_Y.to<double>();
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
        double changeInX = robotX.to<double>() - SPEAKER_BLUE_X.to<double>();
        distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
    }
    else{
        double changeInX = robotX.to<double>() - SPEAKER_RED_X.to<double>();
        distance = sqrtf(pow(changeInX, 2) + pow(changeInY, 2));
    }
    double angle = atan2(pow(PROJECTILE_SPEED, 2) - sqrt(pow(PROJECTILE_SPEED, 4) - (GRAVITY*(GRAVITY*pow(distance, 2) + 2*SPEAKER_HEIGHT*pow(PROJECTILE_SPEED, 2)))), GRAVITY*distance);
    state.targetPivotAngle = units::radian_t(angle);
}

units::velocity::meters_per_second_t Shooter::getProjectileSpeed(bool type){
    if(type){
        return units::velocity::meters_per_second_t(PROJECTILE_SPEED*cos(state.pivotAngle.to<double>()));
    }
    else{
        return units::velocity::meters_per_second_t(PROJECTILE_SPEED*sin(state.pivotAngle.to<double>()));
    }
}
units::radian_t Shooter::getPivotErrorAngle(){
    return state.pivotAngle - state.targetPivotAngle;
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

    builder.AddDoubleProperty(
        "Pivot target angle",
        [this] {return state.targetPivotAngle.to<double>();},
        nullptr
    );

    builder.AddDoubleProperty(
        "Current Pivot angle",
        [this] {return state.pivotAngle.to<double>();},
        nullptr
    );

    builder.AddDoubleProperty(
        "Pivot target angle",
        [this] {return state.targetPivotAngle.convert<units::degree>().to<double>();},
        nullptr
    );

    builder.AddDoubleProperty(
        "Current Pivot angle",
        [this] {return state.pivotAngle.to<double>();},
        nullptr
    );

    builder.AddDoubleProperty("Distance", [this]{return state.distanceFromSpeaker.to<double>();}, nullptr);
}
