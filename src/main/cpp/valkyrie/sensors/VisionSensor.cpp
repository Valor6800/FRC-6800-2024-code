#include "valkyrie/sensors/VisionSensor.h"
#include "valkyrie/sensors/BaseSensor.h"

using namespace valor;
VisionSensor::VisionSensor(frc::TimedRobot *_robot, const char *_name,
                           frc::Pose3d _cameraCoordinates)
    : BaseSensor(_robot, _name) {
    cameraCoordinate = _cameraCoordinates;
    setPipeline(0);
}

VisionSensor::VisionSensor(frc::TimedRobot *_robot, const char *_name,
                           frc::Pose3d _cameraCoordinates,
                           std::unordered_map<int, PipeLines> _myPipes)
    : BaseSensor(_robot, _name) {
    cameraCoordinate = _cameraCoordinates;
    myPipes = _myPipes;
}

void VisionSensor::setPipeline(int pipe) {
    limeTable->PutNumber("pipe", pipe);
    state.pipe = 0;
}
