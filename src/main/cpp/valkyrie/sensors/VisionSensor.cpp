#include "valkyrie/sensors/VisionSensor.h"

using namespace valor;

VisionSensor::VisionSensor(frc::TimedRobot *_robot, std::shared_ptr<nt::NetworkTable> _limeTable, frc::Pose3d _cameraPose) :
    BaseSensor(_robot, "Vision"),
    limeTable(_limeTable),
    cameraPose(_cameraPose)
{
    
}

void VisionSensor::init() {
    
}
