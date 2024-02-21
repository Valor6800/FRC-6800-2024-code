#include "subsystems/Leds.h"

#define LED_LENGTH 62


Leds::Leds(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Leds")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Leds::~Leds() {
}

void Leds::resetState() {
    curr_state = DEFAULT_STATE;
    blinkin.SetPulseTime(InitialPulse);
}

void Leds::init() {

    TrackingBool = nt_shooter->GetBoolean("TrackingTest", false);
    SpooledBool = nt_shooter->GetBoolean("SpooledTest", false);
    //IntakingBool = nt_intake->GetBoolean("TrackingTest", false);
    //DeployedBool = nt_intake->GetBoolean("SpooledTest", false);
    AutoIsONBool = nt_robot->GetBoolean("AutoIsOn", false);

    IntakingBool = nt_feeder->GetBoolean("IntakeTest", false);

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
    //add logic when code added for it
    //Note_DBool = table->GetBoolean("Note_D", false);    
    //PITModeBool = table->GetBoolean("PIT_MODE", false);
    // DeployedBool = nt_intake->GetBoolean("DeplayedTest", false);
    // SpikedBool = nt_intake->GetBoolean("SpikedTest", false);

    TrackingBool = nt_shooter->GetBoolean("TrackingTest", false);
    SpooledBool = nt_shooter->GetBoolean("SpooledTest", false);

    AutoIsONBool = nt_robot->GetBoolean("AutoIsOn", false);

    IntakingBool = nt_feeder->GetBoolean("IntakeTest", false);

    m_animationSelected = m_chooser.GetSelected();

    //selected states(Pulases are picked at random, to test) 
    //intake->state.intakeState == Intake::INTAKE_STATE::SPIKED
    if (SpikedBool){
        curr_state = JAMMED;
    }
    else if(m_animationSelected == kanimone)
    {
        curr_state = NOTE_DETECTED;
    }
    else if(m_animationSelected == kanimtwo)
    {
        curr_state = SPOOLED;
    }
    else if (m_animationSelected == kanimthree)
    {
        curr_state = PIT_MODE;
    }
    //actual logic code
    else if(SpooledBool)
    {
        curr_state = SPOOLED;
    }
    else if(TrackingBool)
    {
        curr_state = TRACKING;
    }
    else if(IntakingBool)
    {
        curr_state = INTAKING;
    }
    else if(DeployedBool)
    {
        curr_state = DEPLOYED;
    }
    else if(AutoIsONBool){
        curr_state = AUTO_IS_ON;
    }
    else if (PITModeBool){
        curr_state = PIT_MODE;
    }
    else if (Note_DBool){
        curr_state = NOTE_DETECTED;
    }
    else
    {
        //default state during teleop
        curr_state = DEFAULT_STATE;
    }

    table->PutBoolean("LED_State",curr_state);
}

void Leds::assignOutputs() 
{
    if(curr_state == JAMMED){
        blinkin.SetPulseTime(JammedPulse);
    }
    else if(curr_state == SPOOLED)
    {
        blinkin.SetPulseTime(SpooledPulse);
    }
    else if(curr_state == TRACKING)
    {
        blinkin.SetPulseTime(TrackingPulse);
    }
    else if(curr_state == INTAKING)
    {
        blinkin.SetPulseTime(IntakingPulse);
    }
    else if(curr_state == DEPLOYED)
    {
        blinkin.SetPulseTime(DeplayedPulse);
    }
    else if(curr_state == AUTO_IS_ON)
    {
        blinkin.SetPulseTime(AutoIsONPulse);
    }
    else if(curr_state == PIT_MODE)
    {
        blinkin.SetPulseTime(PITModePulse);
    }
    else if(curr_state == NOTE_DETECTED)
    {
        blinkin.SetPulseTime(Note_DetectedPulse);
    }
    else
    {
        //default state during teleop
        blinkin.SetPulseTime(Note_NOTDetectedPulse);
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
        [this]{return curr_state;},
        nullptr
    );
}
