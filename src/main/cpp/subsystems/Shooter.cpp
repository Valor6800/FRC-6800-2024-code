#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#define PIVOT_ROTATE_K_VEL 81.36f
#define PIVOT_ROTATE_K_ACC 8136.0f
#define PIVOT_ROTATE_K_P 1.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define PIVOT_CANCODER_GEAR_RATIO 2.0f
#define PIVOT_MAGNET_OFFSET 0.5168f
#define PIVOT_GEAR_RATIO 470.4f
#define PIVOT_REVERSE_LIMIT 68.00f
#define PIVOT_FORWARD_LIMIT 29.0f

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
}

void Shooter::init()
{
    state.pitMode = false;

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
        1.0 / PIVOT_GEAR_RATIO * PIVOT_CANCODER_GEAR_RATIO,
        pivotPID,
        10.0,
        "baseCAN"
    );
    pivotMotors->setupCANCoder(CANIDs::SHOOTER_CANCODER, PIVOT_MAGNET_OFFSET, PIVOT_CANCODER_GEAR_RATIO / 360.0, true, "baseCAN");
    pivotMotors->setRange(0, PIVOT_FORWARD_LIMIT, PIVOT_REVERSE_LIMIT);

    table->PutBoolean("Pit Mode", false);

    //no tracking yet
    table->PutBoolean("SpooledTest", false);
    table->PutBoolean("TrackingTest", false);
    table->PutBoolean("Pit Mode", false);

    SpooledTest = false;
    TrackingTest = false;
    PitModeTest = false;

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || operatorGamepad == nullptr ||
        !driverGamepad->IsConnected() || !operatorGamepad->IsConnected())
        return;

    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        SpooledTest = true;
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } else if (operatorGamepad->GetStartButtonPressed()) {
        SpooledTest = true;
        state.flywheelState = FLYWHEEL_STATE::SPOOLED;
    } else if (operatorGamepad->GetBackButtonPressed()) {
        SpooledTest = false;
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
    bool lastPitMode = state.pitMode;
    state.pitMode = table->GetBoolean("Pit Mode", false);

    if (state.pitMode && !lastPitMode) {
        pivotMotors->setNeutralMode(valor::NeutralMode::Coast);
    } else if (lastPitMode && !state.pitMode) {
        pivotMotors->setNeutralMode(valor::NeutralMode::Brake);
    }
}

void Shooter::assignOutputs()
{
    if (state.flywheelState == NOT_SHOOTING) {
        leftFlywheelMotor.setPower(0.0);
        rightFlywheelMotor.setPower(0.0);
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
    } else if (state.pitMode) {
        PitModeTest = true;
        pivotMotors->setPower(0);
    } else {
        PitModeTest = false;
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>());
    }

    table->PutBoolean("SpooledTest", SpooledTest);
    table->PutBoolean("TrackingTest", TrackingTest);
    table->PutBoolean("Pit Mode", PitModeTest);
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
    
}
