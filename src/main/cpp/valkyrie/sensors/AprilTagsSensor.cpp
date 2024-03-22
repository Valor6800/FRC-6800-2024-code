#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/angle.h"
#include <array>
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include <cmath>
#include <vector>

#define OUTLIER_EDGE 4.0f //meters
#define DP 0.1f
#define VP 0.2f

using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::VisionSensor(robot, name, _cameraPose) {
    setGetter([this](){return getGlobalPose();});
    dp = DP;
    vp = VP;
    limeTable->PutNumber("dp", dp);
    limeTable->PutNumber("vp", vp);
    limeTable->PutNumber("new doubt x", 0);
    limeTable->PutNumber("new doubt y", 0);
}

frc::Pose3d AprilTagsSensor::getGlobalPose() {
    if (!hasTarget()) return frc::Pose3d();
    botPose = limeTable->GetNumberArray("botpose_wpiblue", std::span<double>());
    
    botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());
    if (botToTargetPose.size() == 6) distance = units::meter_t(sqrtf(powf(botToTargetPose[0], 2) + powf(botToTargetPose[1], 2) + powf(botToTargetPose[2], 2)));
    else distance = 0_m;

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

frc::Pose3d AprilTagsSensor::getPoseFromAprilTag() {
    if (!hasTarget()) return frc::Pose3d();

    return frc::Pose3d(
        (units::meter_t) botToTargetPose[0],
        (units::meter_t) botToTargetPose[1],
        (units::meter_t) botToTargetPose[2],
        frc::Rotation3d(
            (units::degree_t) botToTargetPose[3],
            (units::degree_t) botToTargetPose[4],
            (units::degree_t) botToTargetPose[5]
        )
    ); 
}

void AprilTagsSensor::applyVisionMeasurement(frc::SwerveDrivePoseEstimator<4> *estimator, units::velocity::meters_per_second_t speed, bool accept, double doubtX, double doubtY, double doubtRot) {
    if (!hasTarget() || !accept) return;
    dp = limeTable->GetNumber("dp", dp);
    vp = limeTable->GetNumber("vp", vp);
 
    //std::vector<double> botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());
    //if (botToTargetPose.size() == 6) distance = units::meter_t(sqrtf(powf(botToTargetPose[0], 2) + powf(botToTargetPose[1], 2)));
    //else distance = 0_m; return;
    double newDoubtX = doubtX + (distance.to<double>() * dp) + (vp * speed.to<double>());
    double newDoubtY = doubtY + (distance.to<double>() * dp) + (vp * speed.to<double>());
    if (distance >= normalVisionOutlier) return;
    units::millisecond_t totalLatency = getTotalLatency();

    frc::Pose2d tGone = frc::Pose2d{
        currState.ToPose2d().X(),
        currState.ToPose2d().Y(),
        estimator->GetEstimatedPosition().Rotation()
    };
    limeTable->PutNumber("new doubt x", newDoubtX);
    limeTable->PutNumber("new doubt y", newDoubtY);

    if (tGone.X() == 0.0_m || tGone.Y() == 0.0_m)
        return;
    estimator->AddVisionMeasurement(
        tGone,  
        frc::Timer::GetFPGATimestamp() - totalLatency,
        {newDoubtX, newDoubtY, 999999999999.9}
    );
}

int AprilTagsSensor::getTagID(){
    if (!hasTarget()) return -1;

    return limeTable->GetNumber("tid", -1);
}

void AprilTagsSensor::updateMiscValues() { return; }

units::meter_t AprilTagsSensor::getDistanceToRobot() { return 0.0_m; }

units::meter_t AprilTagsSensor::getDistanceToCamera() { return 0.0_m; }

double AprilTagsSensor::getTagCount() { return 0.0; }

double AprilTagsSensor::getTagAmbiguity() { return 0.0; }

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
    builder.AddDoubleProperty("totalLatency", [this] {return getTotalLatency().to<double>();}, nullptr);
    builder.AddDoubleProperty("distanceFromTarget", [this] {return distance.to<double>();}, nullptr);
    builder.AddDoubleProperty("Vision acceptance outlier", [this] {return normalVisionOutlier.to<double>();}, nullptr);
}
