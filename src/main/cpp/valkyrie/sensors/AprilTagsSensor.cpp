#include "valkyrie/sensors/AprilTagsSensor.h"

#define KLIMELIGHT -29.8f //degrees; TODO: MakeModular
using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::VisionSensor(robot, name, _cameraPose) {
   // setGetter([this](){return getGlobalPose();});
}

frc::Pose3d AprilTagsSensor::getGlobalPose() {
    if (!hasTarget() || limeTable == nullptr) return frc::Pose3d();
    std::vector<double> botPose = limeTable->GetNumberArray("botPose", std::span<double>());
    return frc::Pose3d(
        (units::meter_t) botPose[0],
        (units::meter_t) botPose[1],
        (units::meter_t) botPose[2],
        frc::Rotation3d(
            (units::degree_t) botPose[3],
            (units::degree_t) botPose[4],
            (units::degree_t) botPose[5]
        )
    );
}
void AprilTagsSensor::reset() {
    
}

AprilTagsSensor::~AprilTagsSensor() {
    
}

void AprilTagsSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleArrayProperty(
        "globalPos",
        [this]
        {
            std::vector<double> botPose;
            botPose.push_back(currState.X().to<double>());
            botPose.push_back(currState.Y().to<double>());
            botPose.push_back(currState.ToPose2d().Rotation().Degrees().to<double>());
            return botPose;
        },
        nullptr
    );
    builder.AddDoubleProperty("tx", [this]{ return tx;}, nullptr);
    builder.AddDoubleProperty("ty", [this]{ return ty;}, nullptr);
    builder.AddBooleanProperty("hasTarget", [this]{ return hasTarget();}, nullptr);
}
