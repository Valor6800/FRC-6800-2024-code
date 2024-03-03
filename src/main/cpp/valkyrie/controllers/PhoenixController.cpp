#include "valkyrie/controllers/PhoenixController.h"

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

#define FALCON_PIDF_KP 10.0f
#define FALCON_PIDF_KI 0.0f
#define FALCON_PIDF_KD 0.0f
#define FALCON_PIDF_KS 0.1475f // Static friction - maybe 0.05f?

#define FALCON_PIDF_KV 6.0f // RPS cruise velocity
#define FALCON_PIDF_KA 130.0f // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
#define FALCON_PIDF_KJ 650.0f // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

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
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, 6380),
    status(),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    voltageCompenstation(12.0),
    cancoder(nullptr),
    cancoderConversion(1),
    res_position(motor->GetPosition()),
    res_velocity(motor->GetVelocity())
{
    init();
}

PhoenixController::PhoenixController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             double gearRatio,
                                             valor::PIDF pidf,
                                             double voltageComp,
                                             std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, 6380),
    status(),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    voltageCompenstation(voltageComp),
    res_position(motor->GetPosition()),
    res_velocity(motor->GetVelocity())
{
    init(gearRatio, pidf);
}


void PhoenixController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = FALCON_PIDF_KP;
    motionPIDF.I = FALCON_PIDF_KI;
    motionPIDF.D = FALCON_PIDF_KD;
    motionPIDF.error = 0;
    motionPIDF.maxVelocity = FALCON_PIDF_KV;
    motionPIDF.maxAcceleration = FALCON_PIDF_KA;

    init(1, motionPIDF);
}

void PhoenixController::init(double gearRatio, valor::PIDF pidf)
{
    req_position.Slot = 0;
    req_position.UpdateFreqHz = 0_Hz;
    req_velocity.Slot = 0;
    req_velocity.UpdateFreqHz = 0_Hz;
    
    configs::TalonFXConfiguration config;

    config.MotorOutput.Inverted = inverted;
    setNeutralMode(config.MotorOutput, neutralMode);

    // Current limiting configuration
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyTimeThreshold = SUPPLY_TIME_THRESHOLD;

    // Deadband configuration
    config.MotorOutput.DutyCycleNeutralDeadband = FALCON_DEADBAND;

    setPIDF(config, pidf, 0);
    setConversion(config, gearRatio);

    status = motor->GetConfigurator().Apply(config, units::second_t{5});

    wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(motor->GetDeviceID()));
}

void PhoenixController::setupCANCoder(int deviceId, double offset, double _cancoderConversion, bool clockwise, std::string canbus)
{
    cancoderConversion = _cancoderConversion;
    cancoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);
    ctre::phoenix6::configs::MagnetSensorConfigs config;
    config.AbsoluteSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
    config.SensorDirection = clockwise ? signals::SensorDirectionValue::Clockwise_Positive :
                                         signals::SensorDirectionValue::CounterClockwise_Positive;
    config.MagnetOffset = -offset;
    cancoder->GetConfigurator().Apply(config);

    ctre::phoenix6::configs::FeedbackConfigs fx_cfg{};
    fx_cfg.FeedbackRemoteSensorID = cancoder->GetDeviceID();
    fx_cfg.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    fx_cfg.SensorToMechanismRatio = cancoderConversion;
    fx_cfg.RotorToSensorRatio = conversion;
    motor->GetConfigurator().Apply(fx_cfg);
}

double PhoenixController::getCANCoder()
{
    return cancoder ? cancoder->GetAbsolutePosition().GetValueAsDouble() : 0;
}

void PhoenixController::reset()
{
    motor->SetPosition(0_tr);
}

void PhoenixController::setVoltageCompensation(double volts)
{
    voltageCompenstation = volts;
}

void PhoenixController::setEncoderPosition(double position)
{
    motor->SetPosition(units::make_unit<units::turn_t>(position));
}

void PhoenixController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new hardware::TalonFX(canID);
    followerMotor->SetControl(controls::Follower{motor->GetDeviceID(), followerInverted});
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
    config.ReverseSoftLimitEnable = true;
    config.ReverseSoftLimitThreshold = reverse;
    motor->GetConfigurator().Apply(config);
}


void PhoenixController::setPIDF(valor::PIDF _pidf, int slot)
{
    configs::TalonFXConfiguration config{};
    setPIDF(config, _pidf, slot);
    motor->GetConfigurator().Apply(config);
}

void PhoenixController::setPIDF(configs::TalonFXConfiguration & config, valor::PIDF _pidf, int slot)
{
    pidf = _pidf;

    // config.ClosedLoopGeneral.ContinuousWrap = true;

    // Generic PIDF configurations
    // Numerator for closed loop controls will be in volts
    // Feedback and feedforward gains are in volts / rpm of motor, NOT mechanism
    config.Slot0.kP = pidf.P;
    config.Slot0.kI = pidf.I;
    config.Slot0.kD = pidf.D;
    config.Slot0.kV = voltageCompenstation / (maxMotorSpeed / 60.0 * conversion);
    config.Slot0.kS = FALCON_PIDF_KS;

    // Feedforward gain configuration
    if (pidf.aFF != 0) {
        config.Slot0.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
            signals::GravityTypeValue::Elevator_Static :
            signals::GravityTypeValue::Arm_Cosine;
        config.Slot0.kG = pidf.aFF;

        config.Feedback.FeedbackRotorOffset = pidf.aFFTarget;
    }

    // Motion magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = pidf.maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = pidf.maxAcceleration;
    config.MotionMagic.MotionMagicJerk = pidf.maxJerk;
}

/**
 * Input conversion is the gear ratio
 * FROM mechanism rotations TO motor rotations
 * Example from azimuth drivetrain: 1:13.37 = 1/13.37
*/
void PhoenixController::setConversion(double _conversion)
{
    configs::TalonFXConfiguration config{};
    setConversion(config, _conversion);
    motor->GetConfigurator().Apply(config);
}

void PhoenixController::setConversion(configs::TalonFXConfiguration & config, double _conversion)
{
    // Why is this inverted? REV's conversion factor is flipped, so syncrhonize between the two vendors
    conversion = 1.0 / _conversion;
    config.Feedback.SensorToMechanismRatio = conversion; // Now the value is 13.37 in the example!
}

void PhoenixController::setMotorInversion(bool invert){
    motor->SetInverted(invert);
}

bool PhoenixController::getMotorInversion(){
    return motor->GetInverted();
}

double PhoenixController::getCurrent()
{
    return motor->GetTorqueCurrent().GetValueAsDouble();
}

/**
 * Output is in mechanism rotations!
*/
double PhoenixController::getPosition()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_position.Refresh().GetValueAsDouble();
}

/**
 * Output is in mechanism rotations!
*/
double PhoenixController::getSpeed()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_velocity.Refresh().GetValueAsDouble();
}

// Sets signal update rate for position
void PhoenixController::setPositionUpdateFrequency(units::frequency::hertz_t hertz)
{
    res_position.SetUpdateFrequency(hertz);
}

// Sets signal update rate for speed
void PhoenixController::setSpeedUpdateFrequency(units::frequency::hertz_t hertz)
{
    res_velocity.SetUpdateFrequency(hertz);
}

void PhoenixController::setRange(int slot, double min, double max)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = max;
    config.ReverseSoftLimitEnable = true;
    config.ReverseSoftLimitThreshold = min;
    motor->GetConfigurator().Apply(config);
}

/**
 * Set a position in mechanism rotations
*/
void PhoenixController::setPosition(double position)
{
    req_position.Position = units::make_unit<units::turn_t>(position); // Mechanism rotations
    status = motor->SetControl(req_position);
}
/**
 * Set a position in mechanism rotations, use FOC
*/
void PhoenixController::setPosition(double position, bool enableFOC)
{
    req_position.Position = units::make_unit<units::turn_t>(position); // Mechanism rotations
    req_position.EnableFOC = enableFOC;
    status = motor->SetControl(req_position);

}

void PhoenixController::setSpeed(double speed)
{
    req_velocity.Velocity = units::make_unit<units::turns_per_second_t>(speed); // Mechanism rotations
    status = motor->SetControl(req_velocity);
}

void PhoenixController::setPower(double speed)
{
    req_voltage.Output = units::make_unit<units::volt_t>(speed * 12);
    status = motor->SetControl(req_voltage);
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

void PhoenixController::setNeutralMode(configs::MotorOutputConfigs & config, valor::NeutralMode mode)
{
    neutralMode = mode;
    config.NeutralMode = neutralMode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;
}

void PhoenixController::setNeutralMode(valor::NeutralMode mode)
{
    configs::MotorOutputConfigs config{};
    setNeutralMode(config, mode);
    motor->GetConfigurator().Apply(config);
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
    builder.AddDoubleProperty(
        "CANCoder", 
        [this] { return getCANCoder(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqPosition", 
        [this] { return req_position.Position.to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqSpeed", 
        [this] { return req_velocity.Velocity.to<double>(); },
        nullptr);

    builder.AddIntegerProperty(
        "Config Status Code", 
        [this] { return status; },
        nullptr);

    builder.AddIntegerProperty(
        "Magnet Health",
        [this] { return cancoder ? cancoder->GetMagnetHealth().GetValue().value : -1; },
        nullptr);
}
