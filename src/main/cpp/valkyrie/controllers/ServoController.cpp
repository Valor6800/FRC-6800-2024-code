#include "valkyrie/controllers/ServoController.h"

using namespace valor;

ServoController::ServoController(int slot) : frc::PWM(slot, true){

}