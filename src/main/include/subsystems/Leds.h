#pragma once
#include <iostream>
#include <array>

#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/PWM.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "valkyrie/sensors/CurrentSensor.h"
#include "valkyrie/BaseSubsystem.h"
#include "subsystems/Feeder.h"
#include "subsystems/Shooter.h"
#include "subsystems/Climber.h"
#include "Constants.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Leds : public valor::BaseSubsystem
{
public:
    Leds(frc::TimedRobot *robot, Feeder *_feeder, Shooter *_shooter);

    ~Leds();

    void resetState() override;
    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

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

    //this chooser is for testing
    frc::SendableChooser<std::string> m_chooser;
    const std::string kDefaultAnimation = "DefaultAnimation";
    const std::string kanimone = "One";
    const std::string kanimtwo = "Two";
    const std::string kanimthree = "Three";
    std::string m_animationSelected;


    frc::PWM blinkin{DIOPorts::BLINKIN};
    
    LEDState state;

    //-----------STATES BOOLENAS----------------
    bool spooledBool = false;
    bool spikedBool = false;
    bool intakingBool = false;
    bool deployedBool = false;
    bool note_DBool = false;
    bool note_NOTetectedBool = false;
    bool autoIsONBool = false;
    bool PITModeBool = false;
    bool trackingBool = false;
    bool jammedBool = false;

    Feeder* feeder;
    Shooter* shooter;
};
