#include "subsystems/Shooter.h"
#include "Constants.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/DriverStation.h>

#define PIVOT_ROTATE_K_VEL 145.32f
#define PIVOT_ROTATE_K_ACC 2500.0f
#define PIVOT_ROTATE_K_P 1.25f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.42f
#define PIVOT_ROTATE_K_JERK 9999.9f

#define PIVOT_CANCODER_GEAR_RATIO 2.0f
#define PIVOT_MAGNET_OFFSET 0.3272f
#define PIVOT_GEAR_RATIO 219.52f
#define PIVOT_REVERSE_LIMIT 101.00f
#define PIVOT_FORWARD_LIMIT 22.0f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.00005f

#define AMP_ANG 101.0_deg
#define SUBWOOFER_ANG 56.0_deg
#define INTAKE_ANG 62.0_deg
#define PREAMP_ANG 77.5_deg
#define PODIUM_ANG 37.0_deg
#define WING_ANG 26.5_deg
#define POOP_ANG 48.0_deg
#define AUTO_NEAR_ANG 32.5_deg
#define AUTO_FAR_LOW_ANG 27.75_deg
#define AUTO_FAR_HIGH_ANG 27.75_deg
#define AUTO_SUBWOOFER_ANG 58.0_deg

#define AMP_POWER 10.0f // rps
#define LEFT_SHOOT_POWER 60.0f // rps
#define RIGHT_SHOOT_POWER 30.0f // rps
#define LEFT_BLOOP_POWER 32.0f
#define RIGHT_BLOOP_POWER 27.0f

Shooter::Shooter(frc::TimedRobot *_robot, Drivetrain *_drive, frc::AnalogTrigger* _feederBeamBreak, frc::AnalogTrigger* _feederBeamBreak2) :
    valor::BaseSubsystem(_robot, "Shooter"),
    pivotMotors(nullptr),
    feederBeamBreak(_feederBeamBreak),
    feederBeamBreak2(_feederBeamBreak2),
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
    pathplanner::NamedCommands::registerCommand("Set pivot load", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::LOAD;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot podium", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::PODIUM;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot wing", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::WING;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot orbit", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::ORBIT;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Shoot amp", std::move(
        frc2::InstantCommand(
            [this]() {
                state.pivotState = Shooter::PIVOT_STATE::AMP; // WARNING: Rename to AMP after robot_v2 gets merged in
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot far low", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::AUTO_FAR_LOW;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot far high", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::AUTO_FAR_HIGH;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot near", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::AUTO_NEAR;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Reverse flywheels", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.reverseFlywheels = true;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Normal flywheels", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.reverseFlywheels = false;
            }
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Set pivot auto subwoofer", std::move(
        frc2::InstantCommand(
            [this]() {
                state.pivotState = Shooter::PIVOT_STATE::AUTO_SUBWOOFER;
            }
        )
    ).ToPtr());
}

void Shooter::resetState()
{
    state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    state.pivotState = PIVOT_STATE::TRACKING;
    state.calculatingPivotingAngle = units::degree_t{0};
    state.ignoreLoad = false;
    state.otherSide = false;
    state.reverseFlywheels = false;
}

void Shooter::init()
{
    valor::PIDF pivotPID;
    pivotPID.maxVelocity = PIVOT_ROTATE_K_VEL;
    pivotPID.maxAcceleration = PIVOT_ROTATE_K_ACC;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.maxJerk = PIVOT_ROTATE_K_JERK;

    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.aFFType = valor::FeedForwardType::CIRCULAR;

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
    table->PutNumber("Speed Offset Pct", 0.5);
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
        operatorGamepad->GetStartButton() ||
        driverGamepad->GetXButton() ||
        (driverGamepad->GetBButton() && !driverGamepad->GetRightBumper())) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    } else {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (driverGamepad->GetAButton()) {
        if (driverGamepad->GetRightBumper())
            state.pivotState = PIVOT_STATE::FORCE_INTAKE;
        else
            state.pivotState = PIVOT_STATE::SUBWOOFER;
    } else if (driverGamepad->GetBButton()) {
        if (driverGamepad->GetRightBumper())
            state.pivotState = PIVOT_STATE::FORCE_INTAKE;
        else
            state.pivotState = PIVOT_STATE::AMP;
    } else if (driverGamepad->GetXButton()) {
        state.pivotState = PIVOT_STATE::ORBIT;
    } else {
        state.pivotState = PIVOT_STATE::LOAD;
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
    state.tuningOffset = table->GetNumber("Speed Offset Pct", 0.5);

    if ((!feederBeamBreak->GetInWindow() || !feederBeamBreak2->GetInWindow()) && state.pivotState == PIVOT_STATE::LOAD) {
        state.ignoreLoad = true;
    } else 
        state.ignoreLoad = false;

    if (table->GetBoolean("Tuning", false)) {
        state.pivotState = PIVOT_STATE::TUNING;
    }
    units::meter_t xPos = drivetrain->getCalculatedPose_m().X();
    if (
            (xPos > 16.54_m / 2 && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ||
            (xPos < 16.54_m / 2 && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
       ) {
        state.otherSide = true;
    } else {
        state.otherSide = false;
    }
    if (
            (xPos < 5.85_m && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ||
            (xPos > 10.69_m && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
       ) {
        state.insideWing = true;
    } else {
        state.insideWing = false;
    }
    table->PutNumber("On other side", state.otherSide);
    calculatePivotAngle();
}

void Shooter::assignOutputs()
{
    // Do nothing
    if (state.flywheelState == NOT_SHOOTING) {
        leftFlywheelMotor.setPower(0.0);
        rightFlywheelMotor.setPower(0.0);
    } else if (state.pivotState == PIVOT_STATE::TUNING) {
        leftFlywheelMotor.setSpeed(state.tuningSpeed * state.tuningOffset);
        rightFlywheelMotor.setSpeed(state.tuningSpeed);
    } else if (state.pivotState == PIVOT_STATE::AMP) {
        leftFlywheelMotor.setSpeed(AMP_POWER);
        rightFlywheelMotor.setSpeed(AMP_POWER);
    } else if (state.pivotState == PIVOT_STATE::SUBWOOFER || state.pivotState == PIVOT_STATE::DISABLED) {
        leftFlywheelMotor.setSpeed(LEFT_SHOOT_POWER * 0.85);
        rightFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER * 0.85);
    } else if (state.pivotState == PIVOT_STATE::ORBIT) {
        leftFlywheelMotor.setSpeed(LEFT_BLOOP_POWER);
        rightFlywheelMotor.setSpeed(RIGHT_BLOOP_POWER);
    } else {
        if (state.reverseFlywheels) {
            leftFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER);
            rightFlywheelMotor.setSpeed(LEFT_SHOOT_POWER);
        } else {
            leftFlywheelMotor.setSpeed(LEFT_SHOOT_POWER);
            rightFlywheelMotor.setSpeed(RIGHT_SHOOT_POWER);
        }
    }

    if (state.pivotState == PIVOT_STATE::TUNING) {
        pivotMotors->setPosition(state.tuningSetpoint);
    } else if(state.pivotState == PIVOT_STATE::SUBWOOFER){
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::LOAD && !state.ignoreLoad){
        pivotMotors->setPosition(INTAKE_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::PODIUM){
        pivotMotors->setPosition(PODIUM_ANG.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::WING){
        pivotMotors->setPosition(WING_ANG.to<double>() + state.pivotOffset);
    } else if (state.pivotState == PIVOT_STATE::ORBIT || (state.ignoreLoad && state.otherSide)){
        pivotMotors->setPosition(POOP_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::TRACKING || (state.ignoreLoad && !state.otherSide)){
        pivotMotors->setPosition(state.calculatingPivotingAngle.to<double>() + state.pivotOffset);
    } else if(state.pivotState == PIVOT_STATE::AMP){
        pivotMotors->setPosition(AMP_ANG.to<double>() + state.pivotOffset);
    } else if (state.pivotState == PIVOT_STATE::AUTO_FAR_LOW) {
        pivotMotors->setPosition(AUTO_FAR_LOW_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_FAR_HIGH) {
        pivotMotors->setPosition(AUTO_FAR_HIGH_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_NEAR) {
        pivotMotors->setPosition(AUTO_NEAR_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_SUBWOOFER) {
        pivotMotors->setPosition(AUTO_SUBWOOFER_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::FORCE_INTAKE){
        pivotMotors->setPosition(INTAKE_ANG.to<double>());
    } else {
        pivotMotors->setPosition(SUBWOOFER_ANG.to<double>() + state.pivotOffset);
    }
}
 
void Shooter::calculatePivotAngle(){
    double distance = drivetrain->state.distanceFromSpeaker.to<double>();
    distance = fmin(distance, 9.0); // Since the parabola has a positive x^2 term, it'll eventually curve up

    double A = -0.433; // 0;
    double B = 6.42; // 2.23;
    double C = -33.5; // -21.3;
    double D = 89.4; // 78.5;
    double bestPivot = D + (C * distance) + (B * pow(distance, 2)) + (A * pow(distance, 3));
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
