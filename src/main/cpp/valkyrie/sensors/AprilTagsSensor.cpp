#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/angle.h"
#include <array>
#include "units/time.h"
#include <cmath>

#define OUTLIER_EDGE 4.0f //meters

using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::VisionSensor(robot, name, _cameraPose) {
   setGetter([this](){return getGlobalPose();});
}

frc::Pose3d AprilTagsSensor::getGlobalPose() {
    if (!hasTarget()) return frc::Pose3d();
    std::vector<double> botPose = limeTable->GetNumberArray("botpose_wpiblue", std::span<double>());
    
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

void AprilTagsSensor::applyVisionMeasurement(frc::SwerveDrivePoseEstimator<4> *estimator, double outlier, double doubtX, double doubtY, double doubtRot) {
    if (!hasTarget()) return;
 
    std::vector<double> botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());
    if (sqrtf(powf(botToTargetPose[0], 2) + powf(botToTargetPose[1], 2)) >= outlier) return;

    setTotalLatency();

    estimator->AddVisionMeasurement(
        currState.ToPose2d(),  
        frc::Timer::GetFPGATimestamp() - totalLatency,
        {doubtX, doubtY, doubtRot}
    );
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
    builder.AddBooleanProperty("limeTableExist", [this] {return limeTable != nullptr;}, nullptr);
    builder.AddDoubleArrayProperty(
        "cameraPose",
        [this]
        {
            return limeTable->GetNumberArray(
                "camerapose_robotspace",
                std::array<double, 6> {
                    cameraPose.X().to<double>(),
                    cameraPose.Y().to<double>(),
                    cameraPose.Z().to<double>(),
                    cameraPose.Rotation().X().convert<units::degree>().to<double>(),
                    cameraPose.Rotation().Y().convert<units::degree>().to<double>(),
                    cameraPose.Rotation().Z().convert<units::degree>().to<double>()
                }
            );
        },
        nullptr
    );
    builder.AddDoubleProperty("totalLatency", [this] {return totalLatency.to<double>();}, nullptr);
}
