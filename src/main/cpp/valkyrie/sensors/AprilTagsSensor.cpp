#include "valkyrie/sensors/AprilTagsSensor.h"

using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot *_robot, const char *_name) :
    BaseSensor(_robot, _name)
{
    reset();
}

void AprilTagsSensor::reset() {
    
}

void AprilTagsSensor::calculate() {
    prevState = currState;
}