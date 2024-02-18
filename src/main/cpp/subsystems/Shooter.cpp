#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#define PIVOT_ROTATE_K_VEL 10.0f
#define PIVOT_ROTATE_K_ACC 10.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define PIVOT_MIN_ANGLE 28.4f

#define PIVOT_GEAR_RATIO 470.4f
#define PIVOT_REVERSE_LIMIT 35.0f
#define PIVOT_FORWARD_LIMIT 66.0f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.00005f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define LEFT_SHOOT_POWER 50.0f
#define LEFT_SPOOL_POWER 30.0f
#define LEFT_STANDBY_POWER 0.0f

#define RIGHT_SHOOT_POWER 40.0f
#define RIGHT_SPOOL_POWER 30.0f
#define RIGHT_STANDBY_POWER 0.0f

Shooter::Shooter(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Shooter"),
    pivotMotors(nullptr),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, true),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::DISABLED;
    state.leftFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
    state.rightFlywheelTargetVelocity = units::angular_velocity::revolutions_per_minute_t(0);
    state.calculatingPivotingAngle = units::degree_t{0};

    pivotMotors->setEncoderPosition(pivotMotors->getCANCoder() - Constants::shooterPivotOffset() + PIVOT_MIN_ANGLE);
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

    pivotMotors = new valor::PhoenixController(
        CANIDs::PIVOT,
        valor::NeutralMode::Coast,
        false,
        1.0 / PIVOT_GEAR_RATIO * 360,
        pivotPID,
        "baseCAN"
    );
    pivotMotors->setupCANCoder(CANIDs::SHOOTER_CANCODER, 0.5 * 360, true, "baseCAN");
    pivotMotors->setForwardLimit(PIVOT_FORWARD_LIMIT);
    pivotMotors->setReverseLimit(PIVOT_REVERSE_LIMIT);

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
    } else if (operatorGamepad->GetStartButtonPressed()) {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } else if (operatorGamepad->GetBackButtonPressed()) {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (driverGamepad->GetAButton() || operatorGamepad->GetAButton()) {
        state.pivotState = PIVOT_STATE::SUBWOOFER;
    } else if (operatorGamepad->GetBButton()) {
        state.pivotState = PIVOT_STATE::PODIUM;
    } else if (operatorGamepad->GetYButton()) { 
        state.pivotState = PIVOT_STATE::STARTING_LINE;
    } else if (driverGamepad->leftTriggerActive() || operatorGamepad->leftTriggerActive()) {
        state.pivotState = PIVOT_STATE::TRACKING;
    } else {
        state.pivotState = PIVOT_STATE::DISABLED;
    }
    
}

void Shooter::analyzeDashboard()
{
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

    // if(state.pivotState == PIVOT_STATE::SUBWOOFER){
    //     pivotMotors->setPosition(SUBWOOFER_ANG.to<double>());
    // } else if(state.pivotState == PIVOT_STATE::PODIUM){
    //     pivotMotors->setPosition(PODIUM_ANG.to<double>());
    // } else if(state.pivotState == PIVOT_STATE::STARTING_LINE){
    //     pivotMotors->setPosition(STARTING_LINE_ANG.to<double>());
    // } else if(state.pivotState == PIVOT_STATE::TRACKING) {
    //     pivotMotors->setPosition(state.calculatingPivotingAngle.to<double>());
    // } else if (state.pivotState == PIVOT_STATE::DISABLED) {
        pivotMotors->setPower(0);
    // }
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
