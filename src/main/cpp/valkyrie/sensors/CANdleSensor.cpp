#include "valkyrie/sensors/CANdleSensor.h"

#include <string>

using namespace valor;

CANdleSensor::CANdleSensor(frc::TimedRobot *_robot, int ledCount, std::vector<int> segmentCounts, int _canID, std::string _canbus) :
    BaseSensor(_robot, std::string("ID ").append(std::to_string(_canID)).c_str()),
    candle(_canID, _canbus)
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

    for (int segments : segmentCounts) {
        segmentMatrix.push_back({});
        int segmentLEDCount = (ledCount-8)/segments;
        //286
        for (int i = 0; i<segments + 1; i++){
            SegmentSettings newSegment;
            newSegment.recentlyChanged = true;
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
            segmentMatrix.back().push_back(newSegment);
        }
        SegmentSettings allSegment;
        allSegment.startLed = 0;
        allSegment.endLed = ledCount;
        allSegment.activeAnimation = NULL;
        allSegment.currentColor = toRGB(VALOR_GOLD);
        allSegment.recentlyChanged = false;
        allSegment.activeAnimationType = AnimationType::None;
        allSegments.push_back(allSegment);
    }
    setGetter([] { return 0; });
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
    for (uint i = 0; i < segmentMatrix.size(); i++) {
        for (uint j = 0; j <= segmentMatrix[i].size(); j++){
            clearAnimation(i, j);
        }
    }
    for (SegmentSettings allSegment : allSegments) {
        delete allSegment.activeAnimation;
    }
}

void CANdleSensor::setLED(uint led, RGBColor rgb) {
    candle.SetLEDs(
        rgb.red,
        rgb.green,
        rgb.blue,
        0,
        led,
        1
    );
}

void CANdleSensor::setLED(uint led, int color) {
    setLED(led, toRGB(color));
}

void CANdleSensor::setColor(uint layer, uint segment, RGBColor rgb)
{
    if (layer >= segmentMatrix.size()) return;
    segment++;
    if (segment >= segmentMatrix[layer].size()) return;
    segmentMatrix[layer][segment].recentlyChanged = true;
    segmentMatrix[layer][segment].currentColor = rgb;
}

void CANdleSensor::setColor(uint layer, RGBColor rgb)
{
    allSegments[layer].recentlyChanged = true;
    allSegments[layer].currentColor = rgb;
}

void CANdleSensor::setColor(uint layer, uint segment, int color)
{
    setColor(layer, segment, toRGB(color));
}

void CANdleSensor::setAnimation(uint layer, AnimationType animation, RGBColor color, double speed) {
    clearAnimation(0);
    setAnimation(&allSegments[layer], animation, color, speed);
}

void CANdleSensor::setAnimation(uint layer, uint segment, AnimationType animation,RGBColor color, double speed)
{
    if (layer >= segmentMatrix.size()) return;
    segment++;
    if (segment >= segmentMatrix[layer].size()) return;
    clearAnimation(segment);
    setAnimation(&segmentMatrix[layer][segment], animation, color, speed);
}


void CANdleSensor::setAnimation(CANdleSensor::SegmentSettings *segment, AnimationType animation,RGBColor color, double speed)
{
    segment->recentlyChanged = true;
    int brightness = 1;

    segment->activeAnimationType = animation;

    if (animation == AnimationType::ColorFlow){
        segment->activeAnimation = new ctre::phoenix::led::ColorFlowAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            ctre::phoenix::led::ColorFlowAnimation::Forward,
            segment->startLed
        );
    } else if (animation == AnimationType::Fire){
        segment->activeAnimation = new ctre::phoenix::led::FireAnimation(
            brightness,
            speed,
            segment->endLed,
            1,
            1,
            false,
            segment->startLed
        );
    } else if (animation == AnimationType::Larson){
        segment->activeAnimation = new ctre::phoenix::led::LarsonAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            ctre::phoenix::led::LarsonAnimation::Front,
            2,
            segment->startLed
        );
    } else if (animation == AnimationType::Rainbow){
        segment->activeAnimation = new ctre::phoenix::led::RainbowAnimation(
            brightness,
            speed,
            segment->endLed,
            false,
            segment->startLed
        );

    } else if (animation == AnimationType::RgbFade){
        segment->activeAnimation = new ctre::phoenix::led::RgbFadeAnimation(
            brightness,
            speed,
            segment->endLed,
            segment->startLed
        );
    } else if (animation == AnimationType::SingleFade){
        segment->activeAnimation = new ctre::phoenix::led::SingleFadeAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            segment->startLed
        );
    } else if (animation == AnimationType::Strobe){
        segment->activeAnimation = new ctre::phoenix::led::StrobeAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            segment->startLed
        );
    } else if (animation == AnimationType::Twinkle){
        segment->activeAnimation = new ctre::phoenix::led::TwinkleAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            ctre::phoenix::led::TwinkleAnimation::TwinklePercent::Percent100,
            segment->startLed
        );
    } else if (animation == AnimationType::TwinkleOff){
        segment->activeAnimation = new ctre::phoenix::led::TwinkleOffAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->endLed,
            ctre::phoenix::led::TwinkleOffAnimation::TwinkleOffPercent::Percent100,
            segment->startLed
        );
    }
}

void CANdleSensor::clearAnimation(uint layer, uint segment)
{
    if (layer >= segmentMatrix.size()) return;
    segment++;
    if (segment >= segmentMatrix[layer].size()) return;
    if (segmentMatrix[layer][segment].activeAnimation != nullptr) {
        candle.ClearAnimation(0);
        delete segmentMatrix[layer][segment].activeAnimation;
    }
    segmentMatrix[layer][segment].activeAnimationType = AnimationType::None;
}

void CANdleSensor::clearAnimation(uint layer)
{
    if (layer >= segmentMatrix.size()) return;
    clearAnimation(layer, -1);
}

void CANdleSensor::clearAnimation()
{
    for (uint i = 0; i < segmentMatrix.size(); i++)
        clearAnimation(i, -1);
}

CANdleSensor::AnimationType CANdleSensor::getActiveAnimationType(uint layer, uint segment) {
    if (layer >= segmentMatrix.size()) segment = 0;
    segment++;
    if (segment >= segmentMatrix[layer].size()) segment = 0;
    return segmentMatrix[layer][segment].activeAnimationType;
}

CANdleSensor::RGBColor CANdleSensor::getColor(uint layer, uint segment) {
    if (layer >= segmentMatrix.size()) segment = 0;
    segment++;
    if (segment >= segmentMatrix[layer].size()) segment = 0;
    return segmentMatrix[layer][segment].currentColor;
}

void CANdleSensor::reset()
{
    clearAnimation();
    prevState = 0xFFFFFF;
    currState = 0xFFFFFF;
}

void CANdleSensor::calculate()
{
    if (segmentMatrix.size() == 0) return ;
    std::vector<SegmentSettings> & segmentMap = segmentMatrix[0];
    for (int i = segmentMatrix.size() - 1; i >= 0; i--) {
        for (SegmentSettings segment : segmentMatrix[i]) {
            if (segment.activeAnimation != nullptr || segment.currentColor != RGBColor{0, 0, 0}) {
                segmentMap = segmentMatrix[i];
                goto loop_end;
            }
        }
    }
    loop_end:

    for (SegmentSettings& segment : segmentMap) {
        if (segment.activeAnimationType != AnimationType::None) {
            if (segment.activeAnimation) {
                candle.Animate(*(segment.activeAnimation));
            }
            segment.activeAnimationType = AnimationType::None;
        }
        if (segment.recentlyChanged) {
            candle.SetLEDs(
                segment.currentColor.red,
                segment.currentColor.green,
                segment.currentColor.blue,
                0,
                segment.startLed,
                segment.endLed
            );
            segment.recentlyChanged = false;
        }
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
