#include "valkyrie/sensors/CANdleSensor.h"

#include <string>

#define VALOR_GOLD 0xEEA800

CANdleSensor::CANdleSensor(frc::TimedRobot *_robot, int _ledCount, int _segments, int _canID, std::string _canbus) :
    BaseSensor(_robot, std::string("ID ").append(std::to_string(_canID)).c_str()),
    candle(_canID, _canbus),
    ledCount(_ledCount),
    segments(_segments)
{
    wpi::SendableRegistry::AddLW(this, "CANdleSensor", sensorName);

    reset();

    ctre::phoenix::led::CANdleConfiguration config;
    // Should match the type of LED strip connected to the CANdle
    config.stripType = ctre::phoenix::led::LEDStripType::GRB;
    config.brightnessScalar = 0.5;
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    // If the 12V line should be on, off, or modulated (for single LED colors)
    config.vBatOutputMode = ctre::phoenix::led::VBatOutputMode::Off;
    candle.ConfigFactoryDefault(100);
    candle.ConfigAllSettings(config, 100);
    int segmentLEDCount = (ledCount-8)/segments;
    //286
    for (int i = 0; i<segments + 1; i++){
        SegmentSettings newSegment;
        newSegment.recentlyChanged=false;
        newSegment.currentColor = toRGB(VALOR_GOLD);
        newSegment.activeAnimation = NULL;
        newSegment.activeAnimationType = AnimationType::None;
        if (i == 0){
            newSegment.startLed = 0;
            newSegment.endLed = 8;
        }
        else{
            newSegment.startLed = (segmentLEDCount*(i-1)) + 9;
            newSegment.endLed = segmentLEDCount - 1;
        }
        segmentMap[i] = newSegment;
    }
}

CANdleSensor::RGBColor CANdleSensor::toRGB(int color)
{
    RGBColor outColor;
    outColor.red = ((color & 0xFF0000) >> 16);
    outColor.green = ((color & 0x00FF00) >> 8);
    outColor.blue = ((color & 0x0000FF));
    return outColor;
}

CANdleSensor::~CANdleSensor()
{
    for (int i = 0; i <= segments; i++){
        clearAnimation(i);
    }
}

void CANdleSensor::setColor(int segment, RGBColor rgb)
{
    segment++;
    if (segment>=segmentMap.size()){
        return;
    }

    segmentMap[segment].recentlyChanged=true;
    segmentMap[segment].currentColor = rgb;
    calculate();     
}
void CANdleSensor::setColor(RGBColor rgb)
{
    for(int i = 0; i<segmentMap.size(); i++){
        setColor(i, rgb);
    }
}

void CANdleSensor::setColor(int segment, int color)
{
    setColor(segment, toRGB(color));
}

void CANdleSensor::setAnimation(AnimationType animation, RGBColor color, double speed){
    for (auto segment : segmentMap){
        setAnimation(segment.first, animation, color, speed);
    }
}

void CANdleSensor::setAnimation(int segment, AnimationType animation,RGBColor color, double speed)

{
    segment++;
        if (segment>=segmentMap.size()){
        return;
    }
    int brightness = 1;

    if (animation != segmentMap[segment].activeAnimationType) {
        

    segmentMap[segment].recentlyChanged=true;
        clearAnimation(segment);
        // setColor(segment, color);
        // setColor(segmentMap[segment].currentColor);
        segmentMap[segment].activeAnimationType = animation;

        if (animation == AnimationType::ColorFlow){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::ColorFlowAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                ctre::phoenix::led::ColorFlowAnimation::Forward,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::Fire){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::FireAnimation(
                brightness,
                speed,
                segmentMap[segment].endLed,
                1,
                1,
                false,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::Larson){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::LarsonAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                ctre::phoenix::led::LarsonAnimation::Front,
                5,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::Rainbow){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::RainbowAnimation(
                brightness,
                speed,
                segmentMap[segment].endLed,
                false,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::RgbFade){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::RgbFadeAnimation(
                brightness,
                speed,
                segmentMap[segment].endLed,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::SingleFade){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::SingleFadeAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::Strobe){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::StrobeAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::Twinkle){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::TwinkleAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                ctre::phoenix::led::TwinkleAnimation::TwinklePercent::Percent100,
                segmentMap[segment].startLed
            );
        } else if (animation == AnimationType::TwinkleOff){
            segmentMap[segment].activeAnimation = new ctre::phoenix::led::TwinkleOffAnimation(
                segmentMap[segment].currentColor.red,
                segmentMap[segment].currentColor.green,
                segmentMap[segment].currentColor.blue,
                0,
                speed,
                segmentMap[segment].endLed,
                ctre::phoenix::led::TwinkleOffAnimation::TwinkleOffPercent::Percent100,
                segmentMap[segment].startLed
            );
        }
    }

    if (segmentMap[segment].activeAnimation)
        candle.Animate(*(segmentMap[segment].activeAnimation));
}

void CANdleSensor::clearAnimation(int segment)
{
    segmentMap[segment].activeAnimationType = AnimationType::None;

    if (segmentMap[segment].activeAnimation != NULL) {
        candle.ClearAnimation(segmentMap[segment].activeAnimation->GetAnimationIdx());
        delete segmentMap[segment].activeAnimation;
    }
    
}

void CANdleSensor::clearAnimation()
{
    for(auto segment : segmentMap) {
        clearAnimation(segment.first);
    }
}

CANdleSensor::AnimationType CANdleSensor::getActiveAnimationType(int segment) {
    return segmentMap[segment].activeAnimationType;
}

CANdleSensor::RGBColor CANdleSensor::getColor(int segment) {
    return segmentMap[segment].currentColor;
}

void CANdleSensor::reset()
{
    clearAnimation();
    prevState = 0xFFFFFF;
    currState = 0xFFFFFF;
}

void CANdleSensor::calculate()

{
    for(auto segment : segmentMap) {
    if (segment.second.recentlyChanged){
        candle.SetLEDs(
        segment.second.currentColor.red,
        segment.second.currentColor.green,
        segment.second.currentColor.blue,
        0,
        segment.second.startLed,
        segment.second.endLed
    );
       
  }
  segmentMap[segment.first].recentlyChanged=true;
 }

}

void CANdleSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return prevState; },
        nullptr);
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
    builder.AddDoubleProperty(
        "Max Animations",
        [this] {return candle.GetMaxSimultaneousAnimationCount();},
        nullptr);
}
