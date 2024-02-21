#pragma once
#include <iostream>
#include <array>

#include <frc/shuffleboard/ShuffleboardTab.h>

#include <frc/PWM.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

//#include "subsystems/Feeder.h"
#include "valkyrie/sensors/CurrentSensor.h"
#include "valkyrie/BaseSubsystem.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Leds : public valor::BaseSubsystem
{
public:
    Leds(frc::TimedRobot *robot);

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

    frc::SendableChooser<std::string> m_chooser;
    const std::string kDefaultAnimation = "DefaultAnimation";
    const std::string kanimone = "One";
    const std::string kanimtwo = "Two";
    const std::string kanimthree = "Three";
    std::string m_animationSelected;

    static constexpr int kLength = 62;

    frc::PWM blinkin{9};

    //Used Network Tables for other subsystems
    std::shared_ptr<nt::NetworkTable> nt_shooter = nt::NetworkTableInstance::GetDefault().GetTable("Shooter");
    std::shared_ptr<nt::NetworkTable> nt_feeder = nt::NetworkTableInstance::GetDefault().GetTable("Feeder");
    std::shared_ptr<nt::NetworkTable> nt_robot = nt::NetworkTableInstance::GetDefault().GetTable("Robot");


    units::microsecond_t pulseTEST = 1715.0_us;

    units::microsecond_t InitialPulse = 1415.0_us;
    units::microsecond_t JammedPulse = 1405.0_us;
    units::microsecond_t SpooledPulse = 1815.0_us;
    units::microsecond_t TrackingPulse = 1865.0_us;
    units::microsecond_t IntakingPulse = 1875.0_us;
    units::microsecond_t DeplayedPulse = 1635.0_us;
    units::microsecond_t Note_DetectedPulse = 1605.0_us;
    units::microsecond_t Note_NOTDetectedPulse = 1555.0_us; //default state during teleop
    units::microsecond_t AutoIsONPulse = 1755.0_us;
    units::microsecond_t PITModePulse = 1695.0_us;

    double curr_state;
    double prev_state;

    //-----------STATES BOOLENAS----------------
    bool SpooledBool = false;
    bool SpikedBool = false;
    bool IntakingBool = false;
    bool DeployedBool = false;
    bool Note_DBool = false;
    bool Note_NOTetectedBool = false;
    bool AutoIsONBool = false;
    bool PITModeBool = false;
    bool TrackingBool = false;
    bool JammedBool = false;

};