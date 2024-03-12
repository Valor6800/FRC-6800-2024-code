<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> f254cb1 (rebased and cleaned up)
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
candle(_robot, LED_COUNT, 4, CANIDs::CANDLE, "baseCAN") //empty during testing
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    }


void Leds::assignOutputs(){
=======
     
>>>>>>> 3bcf999 (rebased and cleaned up)
=======
    }


void Leds::assignOutputs(){
>>>>>>> c39f3aa (added to assignoutputs)
=======
     
>>>>>>> f254cb1 (rebased and cleaned up)
=======
    }





 


void Leds::assignOutputs(){
>>>>>>> 241a0f2 (added to assignoutputs)
    if (autoIsONBool) {
        candle.setColor(1, CANdleSensor::RGBColor(255, 0, 0)); // Red

    }else{
        candle.setColor(1, CANdleSensor::RGBColor(0, 0, 0)); // black

    }
    if (intakingBool){
        candle.setColor(2, CANdleSensor::RGBColor(0, 255, 0)); // Green


    }else{
        candle.setColor(2, CANdleSensor::RGBColor(0, 0, 0)); // black

    }
    if(trackingBool){
        candle.setColor(3, CANdleSensor::RGBColor(0, 0, 255)); // Blue

        
    }else{
        candle.setColor(3, CANdleSensor::RGBColor(0, 0, 0)); // black

    }
    if(note_DBool){
        candle.setColor(4, CANdleSensor::RGBColor(255, 255, 0)); // Yellow


    }else{
        candle.setColor(4, CANdleSensor::RGBColor(0, 0, 0)); // black

    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
=======
=======
>>>>>>> f254cb1 (rebased and cleaned up)



    }





 


void Leds::assignOutputs(){}
<<<<<<< HEAD
>>>>>>> 3bcf999 (rebased and cleaned up)
=======
}
>>>>>>> c39f3aa (added to assignoutputs)
=======
>>>>>>> f254cb1 (rebased and cleaned up)
=======
}
>>>>>>> 241a0f2 (added to assignoutputs)

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
<<<<<<< HEAD
=======
>>>>>>> bb77256 (rebased and cleaned up)
=======
>>>>>>> f254cb1 (rebased and cleaned up)
