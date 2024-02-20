#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#define PIVOT_ROTATE_K_VEL 81.36f
#define PIVOT_ROTATE_K_ACC 8136.0f
#define PIVOT_ROTATE_K_P 10.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define PIVOT_MIN_ANGLE 28.4f

#define PIVOT_GEAR_RATIO 470.4f
#define PIVOT_REVERSE_LIMIT 68.00f
#define PIVOT_FORWARD_LIMIT 30.0f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.00005f

#define SUBWOOFER_ANG 59.5_deg
#define PODIUM_ANG 39.0_deg
#define WING_ANG 29.0_deg

#define LEFT_SHOOT_POWER 60.0f
#define RIGHT_SHOOT_POWER 30.0f

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
        valor::NeutralMode::Brake,
        false,
        1.0 / PIVOT_GEAR_RATIO * 360,
        pivotPID,
        10.0,
        "baseCAN"
    );
    pivotMotors->setupCANCoder(CANIDs::SHOOTER_CANCODER, 0.5 * 360, true, "baseCAN");
    pivotMotors->setRange(0, PIVOT_FORWARD_LIMIT, PIVOT_REVERSE_LIMIT);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;

    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } else if (operatorGamepad->GetStartButtonPressed()) {
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } else if (operatorGamepad->GetBackButtonPressed()) {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (operatorGamepad->GetAButton()) {// || driverGamepad->GetAButton()) {
        state.pivotState = PIVOT_STATE::SUBWOOFER;
    } else if (operatorGamepad->GetBButton()) {
        state.pivotState = PIVOT_STATE::PODIUM;
    } else if (operatorGamepad->GetYButton()) { 
        state.pivotState = PIVOT_STATE::WING;
    } else if (driverGamepad->leftTriggerActive() || operatorGamepad->leftTriggerActive()) {
        state.pivotState = PIVOT_STATE::TRACKING;
    } else {
        state.pivotState = PIVOT_STATE::DISABLED;
    }
    
}

void Shooter::analyzeDashboard()
{
}

void Shooter::assignOutputs()
{
    if (state.flywheelState == NOT_SHOOTING) {
        leftFlywheelMotor.setPower(0);
        rightFlywheelMotor.setPower(0);
    } else {
        leftFlywheelMotor.setSpeed(LEFT_SHOOT_POWER);
        rightFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER);
    }

    if(state.pivotState == PIVOT_STATE::SUBWOOFER){
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::PODIUM){
        pivotMotors->setPosition(PODIUM_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::WING){
        pivotMotors->setPosition(WING_ANG.to<double>());
    // } else if(state.pivotState == PIVOT_STATE::TRACKING) {
    //     pivotMotors->setPosition(state.calculatingPivotingAngle.to<double>());
    } else {
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>());
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
    
}
