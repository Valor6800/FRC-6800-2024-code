
#include "subsystems/leds.h"
#include <iostream>

#define LED_COUNT 65 //will change based on length

#define CARRIAGE_UPPER_LIMIT 0.89f 
#define CARRIAGE_LOWER_LIMIT 0.0f

#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define WRIST_FORWARD_LIMIT 325.0f
#define WRIST_REVERSE_LIMIT -325.0f

Leds::Leds(frc::TimedRobot *_robot, Feeder *_feeder, Shooter *_shooter) :
valor::BaseSubsystem(_robot, "Leds"), feeder(_feeder), shooter(_shooter),
candle(_robot, LED_COUNT, 4, CANIDs::CANDLE, "") //empty during testing
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();

    state.startedAnimating = std::vector<units::second_t>(4, -1_s);
}


void Leds::init(){
     resetState();

    
    }

void Leds::resetState(){
    candle.clearAnimation();
    candle.setColor(CANdleSensor::RGBColor(0,0,0));
}

void Leds::assessInputs(){}

void Leds::analyzeDashboard(){

    autoIsONBool= robot-> IsAutonomousEnabled();
    intakingBool=feeder-> state.feederState = Feeder::ROLLER_STATE::INTAKE;
    beamBroken= feeder-> isBeamBreakTriggered();
    trackingBool= shooter ->state.pivotState == Shooter::PIVOT_STATE::TRACKING;
}

void Leds::assignOutputs(){


    if (autoIsONBool) {
        candle.setColor(1, CANdleSensor::RGBColor(ledColors::CYAN));
    } else {
        candle.setColor(1, CANdleSensor::RGBColor(ledColors::BLACK)); 
    }

    if (intakingBool) {
        candle.setColor(2, CANdleSensor::RGBColor(ledColors::PURPLE)); 
    } else {
        candle.setColor(2, CANdleSensor::RGBColor(ledColors::BLACK));
    }

    if (trackingBool) {
        candle.setColor(3, CANdleSensor::RGBColor(ledColors::ORANGE));
    } else {
        candle.setColor(3, CANdleSensor::RGBColor(ledColors::BLACK));
    }

    if (beamBroken) {
        candle.setColor(4, CANdleSensor::RGBColor(ledColors::FUCHSIA));
    } else {
        candle.setColor(4, CANdleSensor::RGBColor(ledColors::BLACK));
    }


}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");
    builder.AddBooleanProperty(
        "PIT_MODE",
        [this]{return PIT_MODE;},
        nullptr
    );
    builder.AddDoubleProperty(
        "LED Puisle",
        [this]{return 12.0;},
        nullptr
    );
  
    builder.AddDoubleArrayProperty(
        "Seg 2 colors", 
        [this] {
            std::vector<double> color{(double) candle.getColor(2).red, (double) candle.getColor(2).green, (double) candle.getColor(2).blue};
            return color;
            },
        nullptr
    );

    builder.AddDoubleArrayProperty(
        "Seg 1 colors", 
        [this] {
            std::vector<double> color{(double) candle.getColor(1).red, (double) candle.getColor(1).green, (double) candle.getColor(1).blue};
            return color;
            },
        nullptr
    );
}
