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

#define SPEAKER_Y 5.543042_m
#define OFFSET_FLYWHEEL_FLIPPING 0.4f

#define PIVOT_ROTATE_K_VEL 0.6f
#define PIVOT_ROTATE_K_ACC 50.0f
#define PIVOT_ROTATE_K_P 110.0f //125f
#define PIVOT_ROTATE_K_D 0.1f 
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.40f
#define PIVOT_ROTATE_K_S 0.35f
#define PIVOT_ROTATE_K_JERK 180.0f

#define PIVOT_CANCODER_GEAR_RATIO 1.0f
#define PIVOT_MAGNET_OFFSET -0.628723f
#define PIVOT_GEAR_RATIO 160.9321f 
#define PIVOT_REVERSE_LIMIT 0.25f
#define PIVOT_FORWARD_LIMIT -0.2f

#define FLYWHEEL_ROTATE_K_VEL 75.0f
#define FLYWHEEL_ROTATE_K_ACC 75.0f
#define FLYWHEEL_ROTATE_K_P 0.000007f

#define AMP_ANG 60.0_deg
#define SUBWOOFER_ANG -30.0_deg
#define INTAKE_ANG -72.0_deg
#define PREAMP_ANG -12.5_deg // TODO: kill
#define PODIUM_ANG -53.0_deg
#define WING_ANG -63.5_deg
#define POOP_ANG -40.0_deg
#define BACKSHOT_ANG 69.0_deg
#define AUTO_NEAR_ANG -46.5_deg
#define AUTO_NEAR_FURTHER_ANG -57.0_deg
#define AUTO_FAR_LOW_ANG -63.0_deg
#define AUTO_FAR_WALL_ANG -61.0_deg
#define AUTO_FAR_HIGH_ANG -58.0_deg
#define AUTO_SUBWOOFER_ANG -32.0_deg

#define INTAKE_PIVOT_THRESHOLD -60.0_deg

#define LOWER_ORBIT_DISTANCE_BOUND 3.0f
#define UPPER_ORBIT_DISTANCE_BOUND 9.5f
#define LOWER_ORBIT_SPEED_LEFT 35.0f
#define LOWER_ORBIT_SPEED_RIGHT 30.0f
#define UPPER_ORBIT_SPEED_LEFT 44.0f
#define UPPER_ORBIT_SPEED_RIGHT 39.0f

#define AMP_POWER -10.0f // rps
#define LEFT_SHOOT_POWER 72.0f // rps
#define RIGHT_SHOOT_POWER 46.0f // rps
#define LEFT_FAR_POWER 85.0f
#define RIGHT_FAR_POWER 57.0f
#define LEFT_SUBWOOFER_POWER 60.0f // rps
#define RIGHT_SUBWOOFER_POWER 40.0f // rps

Shooter::Shooter(frc::TimedRobot *_robot, Drivetrain *_drive, frc::AnalogTrigger* _feederBeamBreak, frc::AnalogTrigger* _feederBeamBreak2, valor::CANdleSensor* _leds) :
    valor::BaseSubsystem(_robot, "Shooter"),
    drivetrain(_drive),
    pivotMotors(nullptr),
    feederBeamBreak(_feederBeamBreak),
    feederBeamBreak2(_feederBeamBreak2),
    leftFlywheelMotor(CANIDs::LEFT_SHOOTER_WHEEL, valor::NeutralMode::Coast, true),
    leftFlywheelMotor2(CANIDs::LEFT_SHOOTER_WHEEL2, valor::NeutralMode::Coast, false),
    rightFlywheelMotor(CANIDs::RIGHT_SHOOTER_WHEEL, valor::NeutralMode::Coast, false),
    rightFlywheelMotor2(CANIDs::RIGHT_SHOOTER_WHEEL2, valor::NeutralMode::Coast, true),
    leds(_leds)
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
    pathplanner::NamedCommands::registerCommand("Set pivot far wall", std::move(
        frc2::InstantCommand(
            [this]() {
                state.pivotState = Shooter::PIVOT_STATE::AUTO_FAR_WALL;
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
    pathplanner::NamedCommands::registerCommand("Set pivot near further", std::move(
        frc2::InstantCommand(
            [this]() {
                // shooter->state.isShooting = true;
                state.pivotState = Shooter::PIVOT_STATE::AUTO_NEAR_FURTHER;
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
    pathplanner::NamedCommands::registerCommand("Override flywheels", std::move(
        frc2::InstantCommand(
            [this]() {
                state.flywheelOverride = true;
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
    state.close = false;
    state.far = false;
    state.flywheelOverride = false;
}

void Shooter::init()
{
    valor::PIDF pivotPID;
    pivotPID.maxVelocity = PIVOT_ROTATE_K_VEL;
    pivotPID.maxAcceleration = PIVOT_ROTATE_K_ACC;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.D = PIVOT_ROTATE_K_D;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.maxJerk = PIVOT_ROTATE_K_JERK;
    pivotPID.aFFType = valor::FeedForwardType::CIRCULAR;
    pivotPID.S = PIVOT_ROTATE_K_S;

    valor::PIDF flywheelPID;
    flywheelPID.maxVelocity = FLYWHEEL_ROTATE_K_VEL;
    flywheelPID.maxAcceleration = FLYWHEEL_ROTATE_K_ACC;
    flywheelPID.P = FLYWHEEL_ROTATE_K_P;
    
    leftFlywheelMotor.setConversion(1, 1);
    leftFlywheelMotor.setPIDF(flywheelPID, 0);
    leftFlywheelMotor2.setConversion(1, 1);
    leftFlywheelMotor2.setPIDF(flywheelPID, 0);

    rightFlywheelMotor.setConversion(1, 1);
    rightFlywheelMotor.setPIDF(flywheelPID, 0);
    rightFlywheelMotor2.setConversion(1, 1);
    rightFlywheelMotor2.setPIDF(flywheelPID, 0);

    pivotMotors = new valor::PhoenixController(
        CANIDs::PIVOT_LEAD,
        valor::NeutralMode::Brake,
        true,
        PIVOT_GEAR_RATIO / PIVOT_CANCODER_GEAR_RATIO,
        PIVOT_CANCODER_GEAR_RATIO,
        pivotPID,
        10.0,
        true,
        "baseCAN"
    );
    pivotMotors->setupCANCoder(CANIDs::SHOOTER_CANCODER, PIVOT_MAGNET_OFFSET, false, "baseCAN", ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
    pivotMotors->setRange(0, PIVOT_FORWARD_LIMIT, PIVOT_REVERSE_LIMIT);
    // pivotMotors->setPositionUpdateFrequency(1000_Hz);
    // pivotMotors->setSpeedUpdateFrequency(1000_Hz);
    pivotMotors->enableFOC(true);
    pivotMotors->setPositionUpdateFrequency(250_Hz);

    table->PutNumber("Pivot Setpoint", AMP_ANG.to<double>());
    table->PutNumber("Pivot Power Setpoint", 0);
    table->PutNumber("Speed Setpoint", AMP_POWER);
    table->PutNumber("Speed Offset Pct", 0.5);
    table->PutBoolean("Tuning", false);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;

    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive() && driverGamepad->GetBButton()) {
        state.flywheelState = FLYWHEEL_STATE::REVERSE;
    } else if (driverGamepad->rightTriggerActive() ||
        driverGamepad->leftTriggerActive() ||
        operatorGamepad->GetStartButton() ||
        driverGamepad->GetXButton() ||
        (driverGamepad->GetBButton() && !driverGamepad->GetRightBumper())) {
        state.flywheelState = FLYWHEEL_STATE::SHOOTING;
    } else {
        state.flywheelState = FLYWHEEL_STATE::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (driverGamepad->GetAButton() && driverGamepad->leftTriggerActive()) {
        state.pivotState = PIVOT_STATE::SUBWOOFER;
    } else if (driverGamepad->GetBButton()) {
        if (driverGamepad->GetRightBumper())
            state.pivotState = PIVOT_STATE::FORCE_INTAKE;
        else
            state.pivotState = PIVOT_STATE::AMP;
    } else if (driverGamepad->GetXButton()) {
        state.pivotState = PIVOT_STATE::ORBIT;
    } //else if (driverGamepad->DPadRight()) {
    //     state.pivotState = PIVOT_STATE::BACKSHOT;
    // } 
    else {
        state.pivotState = PIVOT_STATE::LOAD;
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


    auto climberTable = nt::NetworkTableInstance::GetDefault().GetTable("Climber");
    bool ledsAvailable = !climberTable->GetBoolean("Climber overriding leds", false);
    if (ledsAvailable) {
        if(leftFlywheelMotor.getSpeed() > 53){
            leds->setColor(1, valor::CANdleSensor::LIGHT_BLUE);

        }else{
            leds->setColor(1, valor::CANdleSensor::RED);
        }
    }

    int color = 0xAC41FF;
    switch (pivotMotors->getMagnetHealth().value) {
        case 1: 
            color = 0xFF0000;
            break;
        case 2:
            color = 0xFF8C00;
            break;
        case 3:
            color = 0x00FF00;
            break;
    }
    leds->setLED(4, color);

    state.pivotLowered = pivotMotors->getPosition() * 360 < INTAKE_PIVOT_THRESHOLD.to<double>();
    state.close = drivetrain->state.distanceFromSpeaker < 1.80_m;
    state.far = drivetrain->state.distanceFromSpeaker > 5.0_m;

    units::meter_t y = drivetrain->getCalculatedPose_m().Y();
    state.isStraightOrbit = y > SPEAKER_Y;

    if(state.flywheelState == FLYWHEEL_STATE::SHOOTING && state.pivotState != PIVOT_STATE::ORBIT){
        if(fabs(SPEAKER_Y.to<double>() - y.to<double>()) > OFFSET_FLYWHEEL_FLIPPING){
            if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
                state.reverseFlywheels = y >= SPEAKER_Y;
            }
            else{
                state.reverseFlywheels = (y <= SPEAKER_Y);
            }
        }
    }

    else if(state.flywheelState == FLYWHEEL_STATE::SHOOTING && state.pivotState == PIVOT_STATE::ORBIT){
        state.reverseFlywheels = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    }
}

void Shooter::setPivotPosition(double angle)
{
    pivotMotors->setPosition(angle / 360.0);
}

void Shooter::assignOutputs()
{
    // if (table->GetBoolean("Set Break Mode", false)) {
    //     pivotMotors->setNeutralMode(valor::NeutralMode::Brake);
    // } else {
    //     pivotMotors->setNeutralMode(valor::NeutralMode::Coast);
    // }
    // Do nothing
    if (state.flywheelState == NOT_SHOOTING) {
        setFlyweelSpeeds(0.0, 0.0);
    } else if (state.flywheelState == FLYWHEEL_STATE::REVERSE) {
        setFlyweelSpeeds(AMP_POWER, AMP_POWER);
    } else if (state.pivotState == PIVOT_STATE::TUNING) {
        setFlyweelSpeeds(state.tuningSpeed * state.tuningOffset, state.tuningSpeed);
    } else if (state.pivotState == PIVOT_STATE::SUBWOOFER || state.pivotState == PIVOT_STATE::DISABLED || state.close) {
        setFlyweelSpeeds(LEFT_SUBWOOFER_POWER, RIGHT_SUBWOOFER_POWER);
    } else if (state.pivotState == PIVOT_STATE::AMP) {
        setFlyweelSpeeds(0, 0);
    } else if (state.pivotState == PIVOT_STATE::ORBIT) {
        if(state.reverseFlywheels){
            setFlyweelSpeeds(getOrbitSpeeds().second, getOrbitSpeeds().first);
        } else {
            setFlyweelSpeeds(getOrbitSpeeds().first, getOrbitSpeeds().second);
        }

    } else {
        double rightPower = RIGHT_SHOOT_POWER, leftPower = LEFT_SHOOT_POWER;
        if (state.far || state.flywheelOverride) {
            rightPower = RIGHT_FAR_POWER;
            leftPower = LEFT_FAR_POWER;
        }
        if (state.reverseFlywheels) {
            setFlyweelSpeeds(rightPower, leftPower);
        } else {
            setFlyweelSpeeds(leftPower, rightPower);
        }
    }

    // if (state.pivotState == PIVOT_STATE::TUNING) {
    //     pivotMotors->setPower(table->GetNumber("Pivot Power Setpoint", 0));
    // }

    if (state.pivotState == PIVOT_STATE::TUNING) {
        setPivotPosition(state.tuningSetpoint);
    } else if(state.pivotState == PIVOT_STATE::SUBWOOFER){
        setPivotPosition(SUBWOOFER_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::ORBIT){
        setPivotPosition(POOP_ANG.to<double>());
    } else if((state.pivotState == PIVOT_STATE::LOAD && !state.ignoreLoad) || state.otherSide){
        setPivotPosition(INTAKE_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::PODIUM){
        setPivotPosition(PODIUM_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::WING){
        setPivotPosition(WING_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::TRACKING || (state.ignoreLoad && !state.otherSide)){
        setPivotPosition((state.calculatingPivotingAngle).to<double>());
    } else if(state.pivotState == PIVOT_STATE::AMP){
        setPivotPosition(AMP_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::BACKSHOT) {
        setPivotPosition(state.calculatingPivotingAngle.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_FAR_LOW) {
        setPivotPosition(AUTO_FAR_LOW_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_FAR_WALL) {
        setPivotPosition(AUTO_FAR_WALL_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_FAR_HIGH) {
        setPivotPosition(AUTO_FAR_HIGH_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_NEAR) {
        setPivotPosition(AUTO_NEAR_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_NEAR_FURTHER) {
        setPivotPosition(AUTO_NEAR_FURTHER_ANG.to<double>());
    } else if (state.pivotState == PIVOT_STATE::AUTO_SUBWOOFER) {
        setPivotPosition(AUTO_SUBWOOFER_ANG.to<double>());
    } else if(state.pivotState == PIVOT_STATE::FORCE_INTAKE){
        setPivotPosition(INTAKE_ANG.to<double>());
    } else {
        setPivotPosition(SUBWOOFER_ANG.to<double>());
    }
}

std::pair<double, double> Shooter::getOrbitSpeeds()
{
    double distance = drivetrain->state.distanceFromAmp.to<double>();

    if (distance > UPPER_ORBIT_DISTANCE_BOUND) {
        return std::pair<double, double>(UPPER_ORBIT_SPEED_LEFT, UPPER_ORBIT_SPEED_RIGHT);
    } else if (distance < LOWER_ORBIT_DISTANCE_BOUND) {
        return std::pair<double, double>(LOWER_ORBIT_SPEED_LEFT, LOWER_ORBIT_SPEED_RIGHT);
    } else {
        double slopeLeft = (UPPER_ORBIT_SPEED_LEFT - LOWER_ORBIT_SPEED_LEFT) / (UPPER_ORBIT_DISTANCE_BOUND - LOWER_ORBIT_DISTANCE_BOUND);
        double speedLeft = slopeLeft * (distance - LOWER_ORBIT_DISTANCE_BOUND) + LOWER_ORBIT_SPEED_LEFT;
        double slopeRight = (UPPER_ORBIT_SPEED_RIGHT - LOWER_ORBIT_SPEED_RIGHT) / (UPPER_ORBIT_DISTANCE_BOUND - LOWER_ORBIT_DISTANCE_BOUND);
        double speedRight = slopeLeft * (distance - LOWER_ORBIT_DISTANCE_BOUND) + LOWER_ORBIT_SPEED_RIGHT;
        return std::pair<double, double>(speedLeft, speedRight);
    }
}
 
void Shooter::calculatePivotAngle(){
    double distance = drivetrain->state.distanceFromSpeaker.to<double>();
    if (distance > 5.8)
    {
        state.calculatingPivotingAngle = -65.25_deg;
        return ;
    }
    distance = fmin(distance, 9.0); // Since the parabola has a positive x^2 term, it'll eventually curve up


    double A = -0.642;
    double B = 9.2;
    double C = -46.2;
    double D = 20.1; // 78.5;
    if (state.pivotState == PIVOT_STATE::BACKSHOT) {
        A = 1.08;
        B = -12.2;
        C = 48.6;
        D = 1.8;
    }

    double bestPivot = D + (C * distance) + (B * pow(distance, 2)) + (A * pow(distance, 3));
    state.calculatingPivotingAngle = units::degree_t(bestPivot);
}

void Shooter::setFlyweelSpeeds(double leftPower, double rightPower)
{
    if (leftPower == 0.0) {

        leftFlywheelMotor.setPower(0);
        leftFlywheelMotor2.setPower(0);
    } else {
        leftFlywheelMotor.setSpeed(leftPower);
        leftFlywheelMotor2.setSpeed(leftPower);
    }
    if (rightPower == 0.0) {
        rightFlywheelMotor.setPower(0);
        rightFlywheelMotor2.setPower(0);
    } else {
        rightFlywheelMotor.setSpeed(rightPower);
        rightFlywheelMotor2.setSpeed(rightPower);
    }

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
        "pivot setpoint (deg)",
        [this] {return state.calculatingPivotingAngle.to<double>();},
        nullptr
    );

    builder.AddDoubleProperty(
        "pivot setpoint (rot)",
        [this] {return state.calculatingPivotingAngle.to<double>() / 360.0;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Battery voltage",
        [this] {return frc::DriverStation::GetBatteryVoltage();},
        nullptr
    );
    builder.AddDoubleProperty(
        "orbit speed left",
        [this] {return getOrbitSpeeds().first;},
        nullptr
    );
    builder.AddDoubleProperty(
        "orbit speed right",
        [this] {return getOrbitSpeeds().second;},
        nullptr
    );   
    builder.AddDoubleProperty(
        "Overriding flywheels",
        [this] {return state.flywheelOverride;},
        nullptr
    );   
}
