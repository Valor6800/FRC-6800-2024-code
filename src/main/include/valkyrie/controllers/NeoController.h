// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>

#include <iostream>
#include <string>

#include "valkyrie/controllers/BaseController.h"

namespace valor {

class NeoController : public BaseController<rev::CANSparkMax> {
   public:
    NeoController(int, valor::NeutralMode, bool, std::string canbus = "");

    void init();
    void reset();

    double getCurrent();
    double getPosition();
    double getSpeed();

    int getProfile();

    void setEncoderPosition(double position);
    void setVoltageCompensation(double volts) override;
    void setMaxCurrent(double amps);

    void setPosition(double);
    void setSpeed(double);
    void setPower(double);
    void setVoltage(double);

    void setupFollower(int, bool = false);

    void setPIDF(valor::PIDF pidf, int slot);
    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);

    void setConversion(double, double);

    void setMotorInversion(bool);
    bool getMotorInversion();

    void setProfile(int slot);
    void setNeutralMode(valor::NeutralMode nmode);

    void setOpenLoopRamp(double time);

    double getAbsEncoderPosition() override;
    void setupCANCoder(int deviceId, double offset, bool clockwise, std::string canbus = "") override;
    double getCANCoder() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

   private:
    rev::SparkPIDController pidController;
    rev::SparkRelativeEncoder encoder;
    rev::SparkAbsoluteEncoder extEncoder;

    int currentPidSlot;
};
}  // namespace valor
