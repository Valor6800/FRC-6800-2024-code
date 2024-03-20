#pragma once

#include <frc/PWM.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

namespace valor
{

class ServoController
{

public:

    ServoController(int channel, bool registerSendable = true);

    ~ServoController();

    double getPosition();

    double getSpeed();

    void setBounds(units::microsecond_t max, units::microsecond_t deadbandMax, units::microsecond_t center, units::microsecond_t deadbandMin, units::microsecond_t min);

    void setPosition(double pos);
    
    void setSpeed(double speed);

    void InitSendable(wpi::SendableBuilder& builder);

private:

    frc::PWM* servo;
};

}