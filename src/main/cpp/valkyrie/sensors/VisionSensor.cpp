#include "valkyrie/sensors/VisionSensor.h"
#include "valkyrie/sensors/BaseSensor.h"

using namespace valor;

VisionSensor::VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : BaseSensor(robot, name),
            cameraPose(_cameraPose),
            limeTable(nt::NetworkTableInstance::GetDefault().GetTable(name))
{
    wpi::SendableRegistry::AddLW(this, "VisionSensor", sensorName);
    reset();
}

void VisionSensor::reset() {
    tx = 0;
    tv = 0;
    ty = 0;
    pipe = 0;
}

void VisionSensor::setPipe(PipeLines _pipe) {
    if (limeTable == nullptr) return;
    limeTable->PutNumber("pipeline", _pipe);
}

bool VisionSensor::hasTarget() {
    return limeTable != nullptr && tv == 1;
}


units::velocity::meters_per_second_t VisionSensor::getError(int pipe, double kPLimeLight) {
    if (limeTable != nullptr && hasTarget()) {
        double normalizedTx = tx / KLIMELIGHT;
        return units::velocity::meters_per_second_t(((std::fabs(normalizedTx) <= 1 ? normalizedTx : std::copysignf(1.0, normalizedTx) ) * kPLimeLight));
    }
    return units::velocity::meters_per_second_t{0};
}

void VisionSensor::calculate(){
    prevState = currState;

    if (limeTable == nullptr) {
        reset();
        return;
    }

    tv = limeTable->GetNumber("tv", 0.0);
    tx = limeTable->GetNumber("tx", 0.0);
    ty = limeTable->GetNumber("ty", 0.0);
    pipe = limeTable->GetNumber("pipeline", 0);

    currState = getSensor();
    
}