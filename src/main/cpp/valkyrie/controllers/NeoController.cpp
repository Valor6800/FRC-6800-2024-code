#include "valkyrie/controllers/NeoController.h"

using namespace valor;

NeoController::NeoController(int canID,
                                valor::NeutralMode _mode,
                                bool _inverted,
                                std::string canbus) :
    BaseController(new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless), _inverted, _mode, 5676),
    pidController(motor->GetPIDController()),
    encoder(motor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
    extEncoder(motor->GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
    currentPidSlot(0)
{
    init();
}

void NeoController::init()
{
    motor->RestoreFactoryDefaults();
    motor->SetInverted(inverted);
    motor->SetControlFramePeriodMs(0);
    setMaxCurrent(60);
    setVoltageCompensation(12);
    setNeutralMode(neutralMode);
    setRange(0,-1,1);
    valor::PIDF motionPIDF;
    setPIDF(motionPIDF, 0);
    reset();

    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(motor->GetDeviceId()));
}

void NeoController::setMaxCurrent(double current)
{
    motor->SetSmartCurrentLimit(current);
}

void NeoController::reset()
{
    encoder.SetPosition(0);
}

void NeoController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless);
    followerMotor->RestoreFactoryDefaults();
    followerMotor->Follow(*motor, followerInverted);
}

void NeoController::setForwardLimit(double forward)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward);
}

void NeoController::setReverseLimit(double reverse)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, reverse);
}

void NeoController::setPIDF(valor::PIDF pidf, int slot)
{
    pidController.SetP(pidf.P, slot);
    pidController.SetI(pidf.I, slot);
    pidController.SetD(pidf.D, slot);
    pidController.SetFF(1.0 / maxMotorSpeed, slot);
    pidController.SetIZone(0, slot);

    double vel = pidf.maxVelocity * 60.0 / conversion;
    double accel = pidf.maxAcceleration * 60 / conversion;

    pidController.SetSmartMotionMaxVelocity(vel, slot);
    pidController.SetSmartMotionMaxAccel(accel, slot);
    pidController.SetSmartMotionAllowedClosedLoopError(pidf.error, slot);
}

void NeoController::setupCANCoder(int deviceId, double offset, double conversion, bool clockwise, std::string canbus)
{
}

double NeoController::getCANCoder()
{
    return 0;
}

/**
 * Set the conversion rate.
 * Converts between your desired units and rotations of the neo motor shaft (includes gear ratio)
 * @param conversion Conversion rate for position
 */
void NeoController::setConversion(double _conversion)
{
    conversion = _conversion;
    encoder.SetPositionConversionFactor(conversion);
    // convert from minutes to seconds for velocity
    encoder.SetVelocityConversionFactor(conversion / 60.0);
   
}

void NeoController::setMotorInversion(bool invert){
    motor->SetInverted(invert);
}

bool NeoController::getMotorInversion(){
    return motor->GetInverted();
}

void NeoController::setRange(int slot, double min, double max)
{
    pidController.SetOutputRange(min, max, slot);
}

double NeoController::getCurrent()
{
    return motor->GetOutputCurrent();
}

/**
 * Get the position in units (specified by conversion)
 */
double NeoController::getPosition()
{
    return encoder.GetPosition();
}

/**
 * Get the PIDF profile number of the motor
*/
int NeoController::getProfile()
{
    return currentPidSlot;
}

/**
 * Get the speed in units per second (specified by conversion)
 */
double NeoController::getSpeed()
{
    return encoder.GetVelocity();
}

void NeoController::setVoltageCompensation(double volts)
{
    motor->EnableVoltageCompensation(volts);
}

void NeoController::setEncoderPosition(double position)
{
    encoder.SetPosition(position);
}

double NeoController::getAbsEncoderPosition()
{
    return extEncoder.GetPosition();
}

/**
 * Set the position in units (specified by conversion). Example: inches
 */
void NeoController::setPosition(double position)
{
    pidController.SetReference(position, rev::CANSparkMax::ControlType::kSmartMotion, currentPidSlot);
}

void NeoController::setProfile(int profile)
{
    currentPidSlot = profile;
}

/**
 * Set the speed in units per second (specified by conversion). Example: inches per second
 */
void NeoController::setSpeed(double speed)
{
    pidController.SetReference(speed / conversion * 60.0, rev::CANSparkMax::ControlType::kVelocity, currentPidSlot);
}

void NeoController::setPower(double power)
{
    motor->Set(power);
}

void NeoController::setVoltage(double voltage)
{
    pidController.SetReference(voltage, rev::CANSparkMax::ControlType::kVoltage, currentPidSlot);
}

void NeoController::setNeutralMode(valor::NeutralMode mode){  
    motor->SetIdleMode(mode == valor::NeutralMode::Brake ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    neutralMode = mode;
}

void NeoController::setOpenLoopRamp(double time){
    motor->SetOpenLoopRampRate(time);
}

void NeoController::InitSendable(wpi::SendableBuilder& builder)
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
    builder.AddDoubleProperty(
        "Duty Cycle", 
        [this] { return motor->GetAppliedOutput(); },
        nullptr);
    builder.AddDoubleProperty(
        "Voltage", 
        [this] { return motor->GetAppliedOutput() * motor->GetBusVoltage(); },
        nullptr);
    builder.AddBooleanProperty(
        "Inverted", 
        [this] { return inverted; },
        nullptr);
    builder.AddDoubleProperty(
        "duty cycle",
        [this] { return motor->GetAppliedOutput(); },
        nullptr);
}