#include "valkyrie/controllers/ServoController.h"

using namespace valor;

ServoController::ServoController(int channel, bool registerSendable){
    servo = new frc::PWM{channel, registerSendable};
}

ServoController::~ServoController(){
    servo->~PWM();
}

double ServoController::getPosition(){
    return servo->GetPosition();
}

double ServoController::getSpeed(){
    return servo->GetSpeed();
}

void ServoController::setBounds(units::microsecond_t max, units::microsecond_t deadbandMax, units::microsecond_t center, units::microsecond_t deadbandMin, units::microsecond_t min){
    return servo->SetBounds(max, deadbandMax, center, deadbandMin, min);
}

void ServoController::setPosition(double pos){
    servo->SetPosition(pos);
}

void ServoController::setSpeed(double speed){
    servo->SetSpeed(speed);
}

void ServoController::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Position",
        [this] {return ServoController::getPosition();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Speed",
        [this] {return ServoController::getSpeed();},
        nullptr
    );
}