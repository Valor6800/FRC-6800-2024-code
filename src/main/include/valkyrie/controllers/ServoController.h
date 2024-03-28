#pragma once

#include "valkyrie/controllers/BaseController.h"
#include <frc/PWM.h>

namespace valor {

class ServoController : public frc::PWM
{
     public:

    ServoController(int slot);

    void SetRaw(int16_t i);
};
}