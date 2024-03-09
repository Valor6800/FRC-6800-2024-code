#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define PIVOT_ROTATE_K_VEL 67.815f
#define PIVOT_ROTATE_K_ACC 3000.0f
#define PIVOT_ROTATE_K_P 1.5f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f
#define PIVOT_ROTATE_K_JERK 9999.9f

#define PIVOT_CANCODER_GEAR_RATIO 2.0f
#define PIVOT_MAGNET_OFFSET 0.5168f
#define PIVOT_GEAR_RATIO 219.52f
#define PIVOT_REVERSE_LIMIT 80.00f
#define PIVOT_FORWARD_LIMIT 20.0f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.00005f

#define AMP_ANG 55.0_deg
#define SUBWOOFER_ANG 64.5_deg
#define INTAKE_ANG 80.0_deg
#define PODIUM_ANG 37.0_deg
#define WING_ANG 26.5_deg
#define POOP_ANG 48.0_deg

#define AMP_POWER 10.75f // rps
#define LEFT_SHOOT_POWER 60.0f // rps
#define RIGHT_SHOOT_POWER 30.0f // rps
#define LEFT_BLOOP_POWER 32.0f
#define RIGHT_BLOOP_POWER 27.0f

Shooter::Shooter(frc::TimedRobot *_robot, Drivetrain *_drive) :
    valor::BaseSubsystem(_robot, "Shooter"),
    pivotMotors(nullptr),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, true),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Coast, false),
    drivetrain(_drive)
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
                    state.flywheelState = Shooter::FLYWHEEL_STATE::SHOOTING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable shooter", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.flywheelState = Shooter::FLYWHEEL_STATE::SHOOTING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Disable shooter", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.flywheelState = Shooter::FLYWHEEL_STATE::NOT_SHOOTING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Start pivot tracking", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::TRACKING;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot subwoofer", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::SUBWOOFER;
            }
        )
    ).ToPtr());
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::DISABLED;
    state.calculatingPivotingAngle = units::degree_t{0};
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
    pivotPID.maxJerk = PIVOT_ROTATE_K_JERK;

    valor::PIDF flywheelPID;
    flywheelPID.maxVelocity = FLYWHEEL_ROTATE_K_VEL;
    flywheelPID.maxAcceleration = FLYWHEEL_ROTATE_K_ACC;
    flywheelPID.P = FLYWHEEL_ROTATE_K_P;
    
    leftFlywheelMotor.setConversion(1, 1);
    leftFlywheelMotor.setPIDF(flywheelPID, 0);
    rightFlywheelMotor.setConversion(1, 1);
    rightFlywheelMotor.setPIDF(flywheelPID, 0);

    pivotMotors = new valor::PhoenixController(
        CANIDs::PIVOT,
        valor::NeutralMode::Brake,
        false,
        PIVOT_GEAR_RATIO / PIVOT_CANCODER_GEAR_RATIO,
        PIVOT_CANCODER_GEAR_RATIO / 360.0,
        pivotPID,
        10.0,
        false,
        "baseCAN"
    );
    pivotMotors->setupCANCoder(CANIDs::SHOOTER_CANCODER, PIVOT_MAGNET_OFFSET, true, "baseCAN");
    pivotMotors->setRange(0, PIVOT_FORWARD_LIMIT, PIVOT_REVERSE_LIMIT);

    table->PutNumber("Pivot Setpoint", AMP_ANG.to<double>());
    table->PutNumber("Speed Setpoint", AMP_POWER);
    table->PutBoolean("Tuning", false);
    state.pivotOffset = 0.0;

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;

    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive() ||
        driverGamepad->leftTriggerActive() ||
        driverGamepad->GetXButton() ||
        driverGamepad->GetBButton()) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    } else {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (driverGamepad->GetAButton()) {
        state.pivotState = PIVOT_STATE::SUBWOOFER;
    } else if (driverGamepad->GetBButton()) {
        state.pivotState = PIVOT_STATE::AMP;
    } else if (driverGamepad->GetXButton()) {
        state.pivotState = PIVOT_STATE::ORBIT;
    } else if (driverGamepad->leftTriggerActive()) {
        state.pivotState = PIVOT_STATE::TRACKING;
    } else if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper()) {
        state.pivotState = PIVOT_STATE::LOAD;
    } else {
        state.pivotState = PIVOT_STATE::DISABLED;
    }

    //PIVOT OFFSET
    if (operatorGamepad->leftTriggerActive()){
        if (operatorGamepad->DPadUp()){
            state.pivotOffset += 0.25;
        } else if (operatorGamepad->DPadDown()){
            state.pivotOffset -= 0.25;
        } 
    } else {
        state.pivotOffset = 0.0;
    }
}

void Shooter::analyzeDashboard()
{
    state.tuningSetpoint = table->GetNumber("Pivot Setpoint", AMP_ANG.to<double>());
    state.tuningSpeed = table->GetNumber("Speed Setpoint", AMP_POWER);

    if (table->GetBoolean("Tuning", false)) {
        state.pivotState = PIVOT_STATE::TUNING;
    }
    calculatePivotAngle();
}

void Shooter::assignOutputs()
{
    // Do nothing
    if (state.flywheelState == NOT_SHOOTING) {
        leftFlywheelMotor.setPower(0.0);
        rightFlywheelMotor.setPower(0.0);
    } else if (state.pivotState == PIVOT_STATE::TUNING) {
        leftFlywheelMotor.setSpeed(state.tuningSpeed * 0.5);
        rightFlywheelMotor.setSpeed(state.tuningSpeed);
    } else if (state.pivotState == PIVOT_STATE::AMP) {
        leftFlywheelMotor.setSpeed(AMP_POWER);
        rightFlywheelMotor.setSpeed(AMP_POWER);
    } else if (state.pivotState == PIVOT_STATE::SUBWOOFER || state.pivotState == PIVOT_STATE::DISABLED) {
        leftFlywheelMotor.setSpeed(LEFT_SHOOT_POWER * 0.75);
        rightFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER * 0.75);
    } else if (state.pivotState == PIVOT_STATE::ORBIT) {
        leftFlywheelMotor.setSpeed(LEFT_BLOOP_POWER);
        rightFlywheelMotor.setSpeed(RIGHT_BLOOP_POWER);
    } else {
        leftFlywheelMotor.setSpeed(LEFT_SHOOT_POWER);
        rightFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER);
    }

    if (state.pivotState == PIVOT_STATE::TUNING) {
        pivotMotors->setPosition(state.tuningSetpoint);
    } else if(state.pivotState == PIVOT_STATE::SUBWOOFER){
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::LOAD){
        pivotMotors->setPosition(INTAKE_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::PODIUM){
        pivotMotors->setPosition(PODIUM_ANG.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::WING){
        pivotMotors->setPosition(WING_ANG.to<double>() + state.pivotOffset);
    } else if (state.pivotState == PIVOT_STATE::ORBIT){
        pivotMotors->setPosition(POOP_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::TRACKING){
        pivotMotors->setPosition(state.calculatingPivotingAngle.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::AMP){
        pivotMotors->setPosition(AMP_ANG.to<double>() + state.pivotOffset);
    } else {
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>() + state.pivotOffset);
    }
}
 
void Shooter::calculatePivotAngle(){
    double distance = drivetrain->state.distanceFromSpeaker.to<double>();

    double A = 1.41;
    double B = -16.8;
    double C = 74.2;
    double bestPivot = C + (B * distance) + (A * pow(distance, 2));
    state.calculatingPivotingAngle = units::degree_t(bestPivot);
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

    builder.AddDoubleProperty(
        "pivot setpoint",
        [this] {return state.calculatingPivotingAngle.to<double>();},
        nullptr
    );
    
}
