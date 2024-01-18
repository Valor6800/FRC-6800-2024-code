#include "subsystems/Shooter.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define SHOOTER_ANGLE_ADJUST_SPEED 1.0f
#define SHOOTER_MAX_SPEED 2.0f

#define SHOOTER_ROTATE_K_VEL 0.0f
#define SHOOTER_ROTATE_K_ACC_MUL 0.0f
#define SHOOTER_ROTATE_K_F 0.0f
#define SHOOTER_ROTATE_K_P 0.0f
#define SHOOTER_ROTATE_K_I 0.0f
#define SHOOTER_ROTATE_K_D 0.0f
#define SHOOTER_ROTATE_K_ERROR 0.0f
#define SHOOTER_ROTATE_K_AFF 0.0f
#define SHOOTER_ROTATE_K_AFF_POS 0.0f

#define SHOOTER_ROTATE_GEAR_RATIO 1.0f
#define SHOOTER_ROTATE_FORWARD_LIMIT 1.0f
#define SHOOTER_ROTATE_REVERSE_LIMIT 2.0f

#define SHOOTER_STARTING_ANGLE 30.0f
#define SHOOTING_SPEED_SLOWER 0.8f
// #define SHOOTER_ROTATE_S_CURVE_STRENGTH 0.0f

Shooter::Shooter(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Shooter"),
    ShooterAngleControlMotor(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    RightWheelsShootingMotor(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Brake, false),
    LeftWheelShootingMotor(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Brake, false),
    angleAdjustMaxSpeed(SHOOTER_ANGLE_ADJUST_SPEED),
    shootingMaxSpeed(SHOOTER_MAX_SPEED)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Shooter::~Shooter()
{
}

void Shooter::resetState()
{
    state.isShooting = false;
    state.leftSideShooting = false;
    state.rightSideShooting = false;
    state.angle = SHOOTER_STARTING_ANGLE;
    state.shootingSpeed = 0;
}

void Shooter::init()
{
    shooterAngle.velocity = SHOOTER_ROTATE_K_VEL;
    shooterAngle.acceleration = SHOOTER_ROTATE_K_ACC_MUL;
    shooterAngle.F = SHOOTER_ROTATE_K_F;
    shooterAngle.P = SHOOTER_ROTATE_K_P;
    shooterAngle.I = SHOOTER_ROTATE_K_I;
    shooterAngle.D = SHOOTER_ROTATE_K_D;
    shooterAngle.error = SHOOTER_ROTATE_K_ERROR;
    shooterAngle.aFF = SHOOTER_ROTATE_K_AFF;
    shooterAngle.aFFTarget = SHOOTER_ROTATE_K_AFF_POS;

    ShooterAngleControlMotor.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    ShooterAngleControlMotor.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT);
    ShooterAngleControlMotor.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT);
    ShooterAngleControlMotor.setPIDF(shooterAngle, 0);

    resetState();

    table->PutNumber("Shooting Angle Control Max Speed", SHOOTER_ANGLE_ADJUST_SPEED);
    table->PutNumber("Shooting Max Speed", SHOOTER_MAX_SPEED);
    table->PutBoolean("Is Left Side Shooting?", state.leftSideShooting);
    table->PutBoolean("Is Right Side Shooting?", state.rightSideShooting);
    table->PutBoolean("Are We Shooting?", state.isShooting);

}

void Shooter::assessInputs()
{
    if(operatorGamepad->leftStickYActive())
    {
        state.shootingSpeed = operatorGamepad->leftStickY() * SHOOTING_SPEED_SLOWER;
        state.isShooting = true;
        state.rightSideShooting = true;
        state.leftSideShooting = true;
    }
}

void Shooter::analyzeDashboard()
{
    table->PutNumber("Shooting speed", state.shootingSpeed);
    table->PutBoolean("Is shooting?", state.isShooting);
    table->PutBoolean("Is right side shooting?", state.rightSideShooting);
    table->PutBoolean("Is left side shooting?", state.leftSideShooting);
}

void Shooter::assignOutputs()
{
    if(operatorGamepad->leftStickYActive())
    {
        shoot(state.shootingSpeed);
    }
}

void Shooter::shoot(double power)
{
    RightWheelsShootingMotor.setPower(power);
    LeftWheelShootingMotor.setPower(power);
}