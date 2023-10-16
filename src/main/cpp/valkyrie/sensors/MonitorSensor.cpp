#include "valkyrie/sensors/MonitorSensor.h"

#include <math.h>

using namespace valor;

MonitorSensor::MonitorSensor(frc::TimedRobot *_robot, const char *_name) :
    BaseSensor(_robot, _name),
    target_mean(0),
    target_range(0)
{
    wpi::SendableRegistry::AddLW(this, "MonitorSensor", sensorName);
    reset();
}

void MonitorSensor::startRecording()
{
    recording = true;
}

void MonitorSensor::stopRecording()
{
    recording = false;
}

void MonitorSensor::setTargetMean(double _target_mean)
{
    target_mean = _target_mean;
}

void MonitorSensor::setTargetRange(double _target_range)
{
    target_range = _target_range;
}

void MonitorSensor::calculate()
{
    if (recording) {
        history.push_back(getSensor());
        currState = mean();
    }
}

void MonitorSensor::reset()
{
    history.clear();
    currState = 0;
    recording = false;
}

double MonitorSensor::mean()
{
    if (history.size() == 0)
        return 0;
    double sum = 0;
    for (size_t i = 0; i < history.size(); i++) {
        sum += history.at(i);
    }
    return sum / history.size();
}

bool MonitorSensor::getInRange()
{
    return (currState >= (target_mean - target_range)) && (currState <= (target_mean + target_range));
}

void MonitorSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Mean", 
        [this] { return currState; },
        nullptr);
    builder.AddDoubleProperty(
        "Target Mean", 
        [this] { return target_mean; },
        nullptr);
    builder.AddDoubleProperty(
        "Target Range", 
        [this] { return target_range; },
        nullptr);
    builder.AddDoubleProperty(
        "Sample Count", 
        [this] { return history.size(); },
        nullptr);
    builder.AddBooleanProperty(
        "In Range?", 
        [this] { return getInRange(); },
        nullptr);
    builder.AddBooleanProperty(
        "Is Recording?", 
        [this] { return recording; },
        nullptr);
}
