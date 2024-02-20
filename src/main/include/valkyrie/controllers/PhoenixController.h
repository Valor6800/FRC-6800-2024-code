#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>

namespace valor {

class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
public:
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, double gearRatio, valor::PIDF pidf, double voltageComp, std::string _canbus = "");

    void init(double gearRatio, valor::PIDF pidf);
    void init();

    void reset();
    void setNeutralMode(ctre::phoenix6::configs::MotorOutputConfigs& config, valor::NeutralMode mode);
    void setNeutralMode(valor::NeutralMode mode);

    double getPosition();
    double getSpeed();
    double getCurrent();

    void setEncoderPosition(double position);
    void setVoltageCompensation(double volts) override;
    
    void setPosition(double);
    void setSpeed(double);
    void setPower(double);

    void setupFollower(int, bool = false);
    
    void setPIDF(valor::PIDF pidf, int slot);
    void setPIDF(ctre::phoenix6::configs::TalonFXConfiguration& config, valor::PIDF pidf, int slot);

    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);
    
    void setConversion(double);
    void setConversion(ctre::phoenix6::configs::TalonFXConfiguration&, double);

    void setMotorInversion(bool);
    bool getMotorInversion();

    void setProfile(int slot);
    double getAbsEncoderPosition();
    void setupCANCoder(int deviceId, double offset, double conversion, bool clockwise, std::string canbus = "") override;
    double getCANCoder() override;

    /**
     * @brief Prevent the motor from traveling backwards
     * 
     * Restrict the motor from going backwards
     */
    void preventBackwards();
    
    void setOpenLoopRamp(double time);

    void InitSendable(wpi::SendableBuilder& builder);
private:
    valor::PIDF pidf;
    int currentProfile;
    double voltageCompenstation;
    double cancoderConversion;

    ctre::phoenix6::controls::MotionMagicVoltage req_position;
    ctre::phoenix6::controls::VelocityVoltage req_velocity;
    ctre::phoenix6::controls::VoltageOut req_voltage;

    ctre::phoenix::StatusCode status;
    ctre::phoenix6::hardware::CANcoder *cancoder;
};
}
