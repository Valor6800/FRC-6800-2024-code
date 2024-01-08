#include "valkyrie/controllers/PhoenixController.h"

#define FALCON_TICKS_PER_REV 2048

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

#define FALCON_PIDF_KP 4.8f
#define FALCON_PIDF_KI 0.0f
#define FALCON_PIDF_KD 0.1f
#define FALCON_PIDF_KS 0.24f // Add 0.24 V to overcome friction

#define FALCON_PIDF_KV 50.0f // RPS cruise velocity
#define FALCON_PIDF_KA 100.0f // RPS/S acceleration (50/100 = 0.5 seconds to max speed)
#define FALCON_PIDF_KJ 1000.0f // RPS/S^2 jerk (100/1000 = 0.1 seconds to max acceleration)

#define STATOR_CURRENT_LIMIT 60.0f
#define SUPPLY_CURRENT_LIMIT 80.0f
#define SUPPLY_TIME_THRESHOLD 0.75f

#define FALCON_DEADBAND 0.01f

using namespace valor;
using namespace ctre::phoenix6;

PhoenixController::PhoenixController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode)
{
    init();
}

void PhoenixController::init()
{
    motor->GetConfigurator().Apply(configs::TalonFXConfiguration{});
    motor->SetInverted(inverted);
    setNeutralMode(neutralMode);

    // Current limiting configuration
    configs::CurrentLimitsConfigs currentLimits{};
    currentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyTimeThreshold = SUPPLY_TIME_THRESHOLD;
    motor->GetConfigurator().Apply(currentLimits);

    // Deadband configuration
    configs::MotorOutputConfigs config{};
    config.DutyCycleNeutralDeadband = FALCON_DEADBAND;
    motor->GetConfigurator().Apply(config);

    valor::PIDF motionPIDF;
    motionPIDF.P = FALCON_PIDF_KP;
    motionPIDF.I = FALCON_PIDF_KI;
    motionPIDF.D = FALCON_PIDF_KD;
    motionPIDF.F = FALCON_PIDF_KS;
    motionPIDF.error = 0;
    motionPIDF.velocity = FALCON_PIDF_KV;
    motionPIDF.acceleration = FALCON_PIDF_KA;

    setPIDF(motionPIDF, 0);
    reset();

    wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(motor->GetDeviceID()));
}

void PhoenixController::reset()
{
    motor->SetPosition(0_tr);
}

void PhoenixController::setEncoderPosition(double position)
{
    motor->SetPosition(units::make_unit<units::turn_t>(position));
}

void PhoenixController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new hardware::TalonFX(canID);
    followerMotor->SetControl(controls::Follower{motor->GetDeviceID(), followerInverted});
    setNeutralMode(BaseController::neutralMode);
}

void PhoenixController::setForwardLimit(double forward)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = forward;
    motor->GetConfigurator().Apply(config);
}

void PhoenixController::setReverseLimit(double reverse)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = reverse;
    motor->GetConfigurator().Apply(config);
}

void PhoenixController::setPIDF(valor::PIDF _pidf, int slot)
{
    pidf = _pidf;

    configs::ClosedLoopGeneralConfigs closedLoopConfig{};
    closedLoopConfig.ContinuousWrap = true;
    motor->GetConfigurator().Apply(closedLoopConfig);

    // Generic PIDF configurations
    configs::SlotConfigs slotConfig{};
    slotConfig.SlotNumber = slot;
    slotConfig.kP = pidf.P;
    slotConfig.kI = pidf.I;
    slotConfig.kD = pidf.D;
    slotConfig.kV = pidf.F;
    slotConfig.kS = FALCON_PIDF_KS;

    // Feedforward gain configuration
    if (pidf.aFF != 0) {
        slotConfig.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
            signals::GravityTypeValue::Elevator_Static :
            signals::GravityTypeValue::Arm_Cosine;
        slotConfig.kG = pidf.aFF;

        configs::FeedbackConfigs feedForwardConfig{};
        feedForwardConfig.FeedbackRotorOffset = pidf.aFFTarget;
        motor->GetConfigurator().Apply(feedForwardConfig);
    }
    motor->GetConfigurator().Apply(slotConfig);

    // Motion magic configuration
    configs::MotionMagicConfigs magicConfig{};
    magicConfig.MotionMagicCruiseVelocity = pidf.velocity;
    magicConfig.MotionMagicAcceleration = pidf.acceleration;
    magicConfig.MotionMagicJerk = pidf.jerk;
    motor->GetConfigurator().Apply(magicConfig);
}

void PhoenixController::setConversion(double _conversion)
{
    configs::FeedbackConfigs config{};
    config.SensorToMechanismRatio = _conversion;
    motor->GetConfigurator().Apply(config);
    conversion = _conversion;
}

double PhoenixController::getCurrent()
{
    return motor->GetTorqueCurrent().GetValueAsDouble();
}

double PhoenixController::getPosition()
{
    auto& rotorPosSignal = motor->GetRotorPosition();
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return rotorPosSignal.GetValueAsDouble();
}

double PhoenixController::getSpeed()
{
    auto& rotorPosSignal = motor->GetVelocity();
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return rotorPosSignal.GetValueAsDouble();
}

void PhoenixController::setRange(int slot, double min, double max)
{
    
}

void PhoenixController::setPosition(double position)
{
    controls::MotionMagicVoltage request{0_tr};
    request.Slot = currentProfile;
    request.Position = units::make_unit<units::turn_t>(position);
    motor->SetControl(request);
}

void PhoenixController::setSpeed(double speed)
{
    controls::MotionMagicVelocityVoltage request{0_tps};
    request.Slot = currentProfile;
    request.Acceleration = units::make_unit<units::turns_per_second_squared_t>(speed);
    request.Velocity = units::make_unit<units::turns_per_second_t>(speed);
    motor->SetControl(request);
}

void PhoenixController::setPower(double speed)
{
    controls::VoltageOut request{0_V};
    request.Output = units::make_unit<units::volt_t>(speed * 12);
    motor->SetControl(request);
}

void PhoenixController::setProfile(int profile)
{
    currentProfile = profile;
}

void PhoenixController::preventBackwards()
{
    configs::MotorOutputConfigs config{};
    config.PeakReverseDutyCycle = 0;
    motor->GetConfigurator().Apply(config);
}

double PhoenixController::getAbsEncoderPosition()
{
    return 0;
}

void PhoenixController::setNeutralMode(valor::NeutralMode mode)
{
    configs::MotorOutputConfigs config{};
    config.NeutralMode = mode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;
    motor->GetConfigurator().Apply(config);
    
    neutralMode = mode;
}

void PhoenixController::setOpenLoopRamp(double time)
{
    configs::OpenLoopRampsConfigs config{};
    config.DutyCycleOpenLoopRampPeriod = time;
    motor->GetConfigurator().Apply(config);
}

void PhoenixController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Amps", 
        [this] { return getCurrent(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition(); },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed(); },
        nullptr);
    builder.AddBooleanProperty(
        "Inverted", 
        [this] { return inverted; },
        nullptr);

    builder.AddDoubleProperty(
        "Out Volt", 
        [this] { return motor->GetMotorVoltage().GetValueAsDouble(); },
        nullptr);
}