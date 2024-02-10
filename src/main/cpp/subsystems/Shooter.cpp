#include "subsystems/Shooter.h"
#include "Constants.h"
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
#define STARTING_LINE_ANG 60.0_deg

#define LEFT_SHOOT_POWER 66.33f
#define LEFT_SPOOL_POWER 50.0f
#define LEFT_STANDBY_POWER 0.0f

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

#define PROJECTILE_SPEED 7.0f
#define GRAVITY 9.81f
#define SHOOTER_Z_OFFSET 0.0f

#define PIVOT_SUBWOOFER_POSITION 0.0f
#define PIVOT_PODIUM_POSITION 0.00f
#define PIVOT_STARTING_LINE_POSITION 0.00f

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
    
    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);

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

    if (operatorGamepad->GetRightBumperPressed()) {
        state.pivot = PIVOT_STATE::PODIUM;
    }
    else if (operatorGamepad->GetLeftBumperPressed()) { 
        state.pivot = PIVOT_STATE::STARTING_LINE;
    }
    else if (operatorGamepad->GetAButton()) {
        state.pivot = PIVOT_STATE::TRACKING;    
    }
}

void Shooter::analyzeDashboard()
{
    state.pivotAngle = 0.0_deg;

    //NEED PIVOT MOTOR
    // if(state.targetPivotAngle == PivotState::SUBWOOFER){
    //     pivotMotors.setPosition(SUBWOOFER_ANG.to<double>());
    // }
    // else if(state.targetPivotAngle == PivotState::PODIUM){
    //     pivotMotors.setPosition(PODIUM_ANG.to<double>());
    // }
    // else if(state.targetPivotAngle == PivotState::STARTING_LINE){
    //     pivotMotors.setPosition(STARTING_LINE_ANG.to<double>());
    // }
    // else if(state.targetPivotAngle == PivotState::TRACKING){
    //     pivotMotors.setPosition(calculatingPivotingAngle.to<double>());
    // }

    //SHOOTER
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

void Shooter::getLaserTargetPivotAngle(){
    units::meter_t robotX = drivetrain->getPose_m().X();
    units::meter_t robotY = drivetrain->getPose_m().Y();
    double distance = 5;
    state.leftShooterPower = table->GetNumber("Left Shooter Power", LEFT_SHOOT_POWER);
    state.leftSpoolPower = table->GetNumber("Left Shooter Spool", LEFT_SHOOT_SPOOL);
    state.leftStandbyPower = table->GetNumber("Left Shooter Standby", LEFT_SHOOT_STANDBY);

    state.rightShooterPower = table->GetNumber("Right Shooter Power", RIGHT_SHOOT_POWER);
    state.rightSpoolPower = table->GetNumber("Right Shooter Spool", RIGHT_SHOOT_SPOOL);
    state.rightStandbyPower = table->GetNumber("Right Shooter Standby", RIGHT_SHOOT_STANDBY);

    double speakerXOffset = table->GetNumber("Speaker X Offset", SPEAKER_X_OFFSET);
    double speakerYOffset = table->GetNumber("Speaker Y Offset", SPEAKER_Y_OFFSET);
    double speakerZOffset = table->GetNumber("Speaker Z Offset", SPEAKER_Z_OFFSET);

    switch (state.pivot){
        case PIVOT_STATE::PODIUM:
            state.targetPivotAngle = units::radian_t(table->GetNumber("Pivot Podium Angle", PIVOT_PODIUM_POSITION));
            break;
        case PIVOT_STATE::STARTING_LINE:
            state.targetPivotAngle = units::radian_t(table->GetNumber("Pivot Starting Line Angle", PIVOT_STARTING_LINE_POSITION));
            break;
        case PIVOT_STATE::TRACKING:
            getTargetPivotAngle(true);
            break;
        default:
            state.targetPivotAngle = units::radian_t(table->GetNumber("Pivot Subwoofer Angle", PIVOT_SUBWOOFER_POSITION));
            break;
    }
}

void Shooter::assignOutputs()
{

    pivotMotor.setPosition(state.targetPivotAngle.to<double>());

    if (state.flywheelState == FLYWHEEL_STATE::SHOOTING) {
        leftFlywheelMotor.setPower(state.leftShooterPower);
        rightFlywheelMotor.setPower(state.rightShooterPower);
    } else if (state.flywheelState == FLYWHEEL_STATE::SPOOLED) {
        leftFlywheelMotor.setPower(state.leftSpoolPower);
        rightFlywheelMotor.setPower(state.rightSpoolPower);
    } else {
        leftFlywheelMotor.setPower(state.leftStandbyPower);
        rightFlywheelMotor.setPower(state.rightStandbyPower);
    }
}

void Shooter::getTargetPivotAngle(bool laser){
    units::meter_t robotX = drivetrain->getCalculatedPose_m().X();
    units::meter_t robotY = drivetrain->getCalculatedPose_m().Y();
    double distance = sqrt(pow(robotX.to<double>(), 2) + pow(robotY.to<double>(), 2));
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
        double angle = atan2(SPEAKER_HEIGHT + table->GetNumber("Speaker Z Offset", SPEAKER_Z_OFFSET), distance);
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
        double angle = atan2(pow(PROJECTILE_SPEED, 2) - sqrt(pow(PROJECTILE_SPEED, 4) - (GRAVITY*(GRAVITY*pow(distance, 2) + 2*SPEAKER_HEIGHT*pow(PROJECTILE_SPEED, 2)))), GRAVITY*distance);
        state.targetPivotAngle = units::radian_t(angle);
    }
}

void Shooter::getArcTargetPivotAngle(){
    units::meter_t robotX = drivetrain->getPose_m().X();
    units::meter_t robotY = drivetrain->getPose_m().Y();
    double distance = 5;
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

void Shooter::calculateShootingMovingAngle(){
    double accX = drivetrain->getAcceleration(true).to<double>();
    double accY = drivetrain->getAcceleration(false).to<double>();
    double velX = drivetrain->getVelocity(true).to<double>();
    double velY = drivetrain->getVelocity(false).to<double>();
    double posX = drivetrain->getCalculatedPose_m().X().to<double>();
    double posY = drivetrain->getCalculatedPose_m().Y().to<double>();
    double posZ = table->GetNumber("Shooter Z Offset", SHOOTER_Z_OFFSET);

    double time = calculateRootsT(accX, accY, velX, velY, posX, posY);
    double chgX = posX + velX*time + 0.5*accX*time;
    double chgY = posY + velY*time + 0.5*accY*time;
    double h = SPEAKER_HEIGHT - posZ;
    double pivotAngle = atan2(h, sqrt(pow(chgX, 2) + pow(chgY, 2)));
    state.targetPivotAngle = units::angle::radian_t(pivotAngle);
}

units::radian_t Shooter::getPivotErrorAngle(){
    return state.pivotAngle - state.targetPivotAngle;
}


double Shooter::calculateRootsT(double accX, double accY, double velX, double velY, double posX, double posY){
    double A = 0.25 * (pow(accX, 2) + pow(accY, 2));
    double B = (accX * velX) + (accY * velY);
    double C = pow(velX, 2) + (posX*accX) + pow(velY, 2) + (posY*accY) - pow(PROJECTILE_SPEED, 2);
    double D = 2 * (posX * velX + posY * velY);
    double E = pow(posX, 2) + pow(posY, 2);

    return solveQuartic(A, B, C, D, E);
}

double Shooter::solveQuadratic(double a, double b, double c){
    if(a == 0){
        return -c/b;
    }

    double desCrim = pow(b, 2) - 4*a*c;

    if(desCrim < 0){
        return -1;
    }

    double x1 = (-b + sqrt(desCrim)) / (2*a);
    double x2 = (-b - sqrt(desCrim)) / (2*a);
    return x1, x2;
}

double Shooter::solveCubic(double a, double b, double c, double d){
    if(a == 0){
        return solveQuadratic(b, c, d);
    }

    double p = -b/(3*a);
    double q = pow(p, 3) + (b*c - 3*a*d)/(6*pow(a, 2));
    double r = c/(3*a);

    double r1 = pow(pow(q, 2) + pow(r - pow(p, 2), 3), 0.5);
    double r2 = -pow(pow(q, 2) + pow(r - pow(p, 2), 3), 0.5);

    double q1 = pow(q + r1, 1.0/3.0);
    double q2 = pow(q - r1, 1.0/3.0);

    double q3 = pow(q - r2, 1.0/3.0);
    double q4 = pow(q + r2, 1.0/3.0);

    double root1 = q1 + q2 + p;
    double root2 = q3 + q4 + p;

    return root1, root2;
}

double Shooter::solveQuartic(double a1, double b1, double c1, double d1, double e1){
    if(a1 == 0){
        return solveCubic(b1, c1, d1, e1);
    }
    
    double a = b1/a1;
    double b = c1/a1;
    double c = d1/a1;
    double d = e1/a1;

    double p = b - (3*pow(a, 2))/8;
    double q = c - ((a*b)/2) + (pow(a, 3)/8);
    double r = d - ((a*c)/4) + ((pow(a, 2)*b)/16) - (3*pow(a, 4)/256);
    double g1, g2 = solveCubic(1, 2*p, pow(p, 2) - 4*r, -pow(q, 2));

    if(g1 < 0 && g2 < 0){
        return -1.0;
    }

    double s1 = g1 - (2*(p + g1 + (q/sqrt(g1))));
    double s2 = g2 - (2*(p + g2 + (q/sqrt(g2))));

    if(s1 < 0 && s2 < 0){
        return -1.0;
    }

    double x1 = (-sqrt(g1) + sqrt(s1))/(2) - (a/4);
    double x2 = (-sqrt(g1) - sqrt(s1))/(2) - (a/4);
    double x3 = (sqrt(g1) + sqrt(s1))/(2) - (a/4);
    double x4 = (sqrt(g1) - sqrt(s1))/(2) - (a/4);

    double x11 = (-sqrt(g2) + sqrt(s2))/(2) - (a/4);
    double x12 = (-sqrt(g2) - sqrt(s2))/(2) - (a/4);
    double x13 = (sqrt(g2) + sqrt(s2))/(2) - (a/4);
    double x14 = (sqrt(g2) - sqrt(s2))/(2) - (a/4);

    double solutions[] = {x1, x2, x3, x4, x11, x12, x13, x14};
    double smallest = 900;
    for(int i = 0; i <= 4; i++){
        if(solutions[i] < smallest){
            smallest = solutions[i];
        }
    }
    return smallest;
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
