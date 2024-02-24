#include "subsystems/Leds.h"

#define pulseTEST 1715.0_us 
#define InitialPulse 1415.0_us // Breath, Red
#define JammedPulse 1405.0_us // Heartbeat, Grey
#define SpooledPulse 1815.0_us // Solid, Orange
#define TrackingPulse 1865.0_us // Solid, Lime
#define IntakingPulse 1875.0_us // Solid, Dark Green
#define DeplayedPulse 1635.0_us // Two-color pattern: Heartbeat Fast
#define Note_DetectedPulse 1605.0_us // Two-color pattern: Chasing Light
#define Note_NOTDetectedPulse 1555.0_us // Default state during teleop LEDs: One color, Breath Fast
#define AutoIsONPulse 1755.0_us // Twinkles (Color 1 and Color 2)
#define PITModePulse 1695.0_us // Sparkle, Color 2 on Color 1


Leds::Leds(frc::TimedRobot *_robot, Feeder *_feeder, Shooter *_shooter) : 
valor::BaseSubsystem(_robot, "Leds"), feeder(_feeder), shooter(_shooter) // new Shooter(robot, "shooter")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Leds::~Leds() {
}

void Leds::resetState() {
    currState = DEFAULT_STATE;
    blinkin.SetPulseTime(InitialPulse);
}

void Leds::init() {

    autoIsONBool = robot->IsAutonomousEnabled();

    intakingBool = feeder->isIntake();
    note_DBool = feeder->isBeamBreakTriggered();

    trackingBool = shooter->getTrackingState();
    spooledBool = shooter->getSpooledState();
    PITModeBool = shooter->getPitModeState();

    table->PutNumber("LED_State", DEFAULT_STATE);

    //can add more animations to choose from
    m_chooser.SetDefaultOption(kDefaultAnimation, kDefaultAnimation);
    m_chooser.AddOption(kanimone, kanimone);
    m_chooser.AddOption(kanimtwo, kanimtwo);
    m_chooser.AddOption(kanimthree, kanimthree);
    frc::SmartDashboard::PutData("Aanimation Modes", &m_chooser);

    resetState();
}

void Leds::assessInputs() {
}

void Leds::analyzeDashboard() {
    autoIsONBool = robot->IsAutonomousEnabled();

    intakingBool = feeder->isIntake();
    note_DBool = feeder->isBeamBreakTriggered();

    trackingBool = shooter->getTrackingState();
    spooledBool = shooter->getSpooledState();
    PITModeBool = shooter->getPitModeState();

    m_animationSelected = m_chooser.GetSelected();

    
    spikedBool = feeder->state.intakeState == Feeder::ROLLER_STATE::SPIKED;

    //chooser for animations after spike
    if (spikedBool){
        currState = JAMMED;
        LEDState::JAMMED;
    }
    else if(m_animationSelected == kanimone)
    {
        currState = NOTE_DETECTED;
        LEDState::NOTE_DETECTED;
    }
    else if(m_animationSelected == kanimtwo)
    {
        currState = SPOOLED;
        LEDState::SPOOLED;
    }
    else if (m_animationSelected == kanimthree)
    {
        currState = PIT_MODE;
        LEDState::PIT_MODE;
    }
    //actual logic code during the game
    else if(spooledBool)
    {
        currState = SPOOLED;
        LEDState::SPOOLED;
    }
    else if(trackingBool)
    {
        currState = TRACKING;
        LEDState::TRACKING;
    }
    else if(intakingBool)
    {
        currState = INTAKING;
        LEDState::INTAKING;
    }
    else if(deployedBool)
    {
        currState = DEPLOYED;
        LEDState::DEPLOYED;
    }
    else if(autoIsONBool){
        currState = AUTO_IS_ON;
        LEDState::AUTO_IS_ON;
    }
    else if (PITModeBool){
        currState = PIT_MODE;
        LEDState::PIT_MODE;
    }
    else if (note_DBool){
        currState = NOTE_DETECTED;
        LEDState::NOTE_DETECTED;
    }
    else
    {
        //default state during teleop
        currState = DEFAULT_STATE;
    }
}

void Leds::assignOutputs() 
{
     switch(currState) {
        case JAMMED:
            blinkin.SetPulseTime(JammedPulse);
            break;
        case SPOOLED:
            blinkin.SetPulseTime(SpooledPulse);
            break;
        case TRACKING:
            blinkin.SetPulseTime(TrackingPulse);
            break;
        case INTAKING:
            blinkin.SetPulseTime(IntakingPulse);
            break;
        case DEPLOYED:
            blinkin.SetPulseTime(DeplayedPulse);
            break;
        case AUTO_IS_ON:
            blinkin.SetPulseTime(AutoIsONPulse);
            break;
        case PIT_MODE:
            blinkin.SetPulseTime(PITModePulse);
            break;
        case NOTE_DETECTED:
            blinkin.SetPulseTime(Note_DetectedPulse);
            break;
        default:
            //default state during teleop
            blinkin.SetPulseTime(Note_NOTDetectedPulse);
            break;
    }
}

void Leds::InitSendable(wpi::SendableBuilder& builder) {
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
    builder.AddDoubleProperty(
        "LED_State",
        [this]{return currState;},
        nullptr
    );
}

