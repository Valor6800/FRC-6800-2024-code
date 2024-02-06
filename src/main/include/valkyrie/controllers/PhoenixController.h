#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <string>

namespace valor {

class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
public:
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, double gearRatio, valor::PIDF pidf, std::string _canbus = "");

    void init(double gearRatio, valor::PIDF pidf);
    void init();

    void reset();
    void setNeutralMode(valor::NeutralMode mode);

    double getPosition();
    double getSpeed();
    double getCurrent();

    void setEncoderPosition(double position);
    
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

    ctre::phoenix6::controls::MotionMagicVoltage req_position;
    ctre::phoenix6::controls::VelocityVoltage req_velocity;
    ctre::phoenix6::controls::VoltageOut req_voltage;

    ctre::phoenix::StatusCode status;
};
}
