#include "subsystems/Leds.h"
#include <iostream>

#define LED_COUNT 288

#define CARRIAGE_UPPER_LIMIT 0.89f 
#define CARRIAGE_LOWER_LIMIT 0.0f

#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define WRIST_FORWARD_LIMIT 325.0f
#define WRIST_REVERSE_LIMIT -325.0f

Leds::Leds(frc::TimedRobot *_robot, Feeder *_feeder, Drivetrain *_drivetrain, Shooter *_shooter) :
valor::BaseSubsystem(_robot, "Leds"), feeder(_feeder), shooter(_shooter),
candle(_robot, LED_COUNT, 2, CANIDs::CANDLE, "baseCAN")
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


    intakingBool=feeder->isIntake();
    note_DBool= feeder-> isBeamBreakTriggered();

    trackingBool = shooter->getTrackingState();
 
    if (spooledBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    else if (trackingBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    else if (intakingBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    else if (deployedBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    else if (autoIsONBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    
    else if (note_DBool){
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }
    else{
        candle.setColor(CANdleSensor::RGBColor(0,0,0));
    }

}

void Leds::assignOutputs(){}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");
    builder.AddBooleanProperty(
        "PIT_MODE",
        //yet need to figure this out
        [this]{return PIT_MODE;},
        nullptr
    );
    builder.AddDoubleProperty(
        "LED Puisle",
        //yet need to figure this out
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


