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
    setGlobalPosition();
}

void AprilTagsSensor::setGlobalPosition() {
    std::vector<double> botPose = getSensor().limeTable->GetNumberArray("botpose", std::span<double>() );
    getSensor().globalPose = frc::Pose3d(
      (units::length::meter_t) botPose[0],
      (units::length::meter_t) botPose[1],
      (units::length::meter_t) botPose[2],
      frc::Rotation3d(
        (units::degree_t) botPose[3],
        (units::degree_t) botPose[4],
        (units::degree_t) botPose[5]
      )
    );
}


