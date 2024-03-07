#pragma once
#include <functional>
#include <frc/TimedRobot.h>
#include "valkyrie/sensors/CANdleSensor.h"
#include "Constants.h"

#include "valkyrie/BaseSubsystem.h"
#include "subsystems/Feeder.h"
#include "subsystems/Shooter.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>




#include <vector>
class Leds : public valor::BaseSubsystem
{
    public:
        Leds(frc::TimedRobot *robot, Feeder *feeder, Drivetrain *drivetrain, Shooter *shooter);
        CANdleSensor::RGBColor value;
        
        void init();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();
        void InitSendable(wpi::SendableBuilder& builder) override;

        struct State{
            std::vector<units::second_t> startedAnimating; 
        } state;


      enum LEDState
    {
        DEFAULT_STATE,
        PIT_MODE,
        AUTO_IS_ON,
        JAMMED,
        DEPLOYED,
        INTAKING,
        NOTE_DETECTED,
        SPOOLED,
        TRACKING
    };


        
    private:
        bool spooledBool=false;
        bool spikedBool=false;
        bool intakingBool=false;
        bool deployedBool=false;
        bool note_DBool=false;
        bool note_NOTdetected= false;
        bool autoIsONBool = false;
        bool PITModeBool=false;
        bool trackingBool= false;
        bool jammedBool = false;

        CANdleSensor candle;
        Feeder *feeder;
        Shooter *shooter;
        
};
