#include "valkyrie/controllers/FalconController.h"

#define FALCON_TICKS_PER_REV 2048

using namespace valor;
using namespace ctre::phoenix6;

FalconController::FalconController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode)
{
    init();
}

void FalconController::init()
{
    motor->GetConfigurator().Apply(configs::TalonFXConfiguration{});
    motor->SetInverted(inverted);
    setNeutralMode(neutralMode);

    configs::CurrentLimitsConfigs currentLimits;
    currentLimits.StatorCurrentLimit = 60.0;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 80.0;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyTimeThreshold = 0.75;
    motor->GetConfigurator().Apply(currentLimits);

    configs::MotorOutputConfigs config;
    config.DutyCycleNeutralDeadband = 0.01;
    motor->GetConfigurator().Apply(config);

    valor::PIDF motionPIDF;
    setPIDF(motionPIDF, 0);
    reset();

    wpi::SendableRegistry::AddLW(this, "FalconController", "ID " + std::to_string(motor->GetDeviceID()));
}

void FalconController::reset()
{
    motor->SetSelectedSensorPosition(0);
}

void FalconController::setEncoderPosition(double position)
{
    motor->SetSelectedSensorPosition(position / conversion * FALCON_TICKS_PER_REV, 0);
}

void FalconController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new TalonFX(canID);
    followerMotor->Follow(*motor);
    if (followerInverted) {
        followerMotor->SetInverted(!motor->GetInverted());
    }
    setNeutralMode(BaseController::neutralMode);
}

void FalconController::setForwardLimit(double forward)
{
    double rawForward = forward / conversion * FALCON_TICKS_PER_REV;
    configs::SoftwareLimitSwitchConfigs config;
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = rawForward;
    motor->GetConfigurator().Apply(config);
}

void FalconController::setReverseLimit(double reverse)
{
    double rawReverse = reverse / conversion * FALCON_TICKS_PER_REV;
    configs::SoftwareLimitSwitchConfigs config;
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = rawReverse;
    motor->GetConfigurator().Apply(config);
}

void FalconController::setPIDF(valor::PIDF _pidf, int slot)
{
    pidf = _pidf;
    configs::Slot0Configs config{};
    config.kP = pidf.P;
    config.kI = pidf.I;
    config.kD = pidf.D;
    config.kV = pidf.F * (1023.0 / 7112.0);
    motor->GetConfigurator().Apply(config);
    configs::
    
    motor->ConfigAllowableClosedloopError(slot, pidf.error * FALCON_TICKS_PER_REV / conversion);
    double vel = pidf.velocity / 10.0 * FALCON_TICKS_PER_REV / conversion;
    motor->ConfigMotionCruiseVelocity(vel);
    motor->ConfigMotionAcceleration(vel / pidf.acceleration);
    motor->ConfigMotionSCurveStrength(pidf.sCurveStrength);
}

void FalconController::setConversion(double _conversion)
{
    conversion = _conversion;
}

double FalconController::getCurrent()
{
    return motor->GetOutputCurrent();
}

/**
 * Get the position in units (specified by conversion)
 */
double FalconController::getPosition()
{
    auto& rotorPosSignal = motor->GetRotorPosition();
    // @TODO Use FPGA - latency to identify timestamp of calculation
    units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return rotorPosSignal.GetValueAsDouble() * conversion / FALCON_TICKS_PER_REV;
}

double FalconController::getSpeed()
{
    auto& rotorPosSignal = motor->GetVelocity();
    // @TODO Use FPGA - latency to identify timestamp of calculation
    units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return rotorPosSignal.GetValueAsDouble() * 10 * conversion / FALCON_TICKS_PER_REV;
}

void FalconController::setRange(int slot, double min, double max)
{
    
}

void FalconController::setPosition(double position)
{
    if (pidf.aFF != 0) {
        double horizontalOffset = getPosition() - pidf.aFFTarget;
        double scalar = std::cos(horizontalOffset * M_PI / 180.0);
        motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV, DemandType_ArbitraryFeedForward, scalar * pidf.aFF);
    } else
        motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV);
}

void FalconController::setSpeed(double speed)
{
    motor->Set(ControlMode::Velocity, speed / 10 / conversion * FALCON_TICKS_PER_REV);
}

void FalconController::setPower(double speed)
{
    // @TODO built in voltage compenstation and saturation
    motor->Set(ControlMode::PercentOutput, speed);
}

void FalconController::setProfile(int profile)
{
    motor->SelectProfileSlot(profile, 0);
}

void FalconController::preventBackwards()
{
    configs::MotorOutputConfigs config;
    config.PeakReverseDutyCycle = 0;
    motor->GetConfigurator().Apply(config);
}

double FalconController::getAbsEncoderPosition()
{
    return 0;
}

void FalconController::setNeutralMode(valor::NeutralMode mode)
{
    configs::MotorOutputConfigs config;
    config.NeutralMode = mode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;
    motor->GetConfigurator().Apply(config);
    
    neutralMode = mode;
}

void FalconController::setOpenLoopRamp(double time)
{
    configs::OpenLoopRampsConfigs config;
    config.DutyCycleOpenLoopRampPeriod = time;
    motor->GetConfigurator().Apply(config);
}

void FalconController::InitSendable(wpi::SendableBuilder& builder)
{
    BaseController::InitSendable(builder);
    builder.AddDoubleProperty(
        "Out Volt", 
        [this] { return motor->GetMotorOutputVoltage(); },
        nullptr);
}