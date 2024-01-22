#include "valkyrie/sensors/GamePieceSensor.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "units/length.h"
#include <cmath>
#include <vector>
using namespace valor;

GamePieceSensor::GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator) : valor::VisionSensor(robot, name, _cameraPose),
    estimator(estimator)
{
    setGetter([this](){return getGlobalPose();});
}

frc::Pose3d GamePieceSensor::getGlobalPose() {
    if (!hasTarget() || estimator->GetEstimatedPosition().X() == 0.0_m || estimator->GetEstimatedPosition().Y() == 0.0_m) return frc::Pose3d();
    
    units::degree_t tx = (units::degree_t) limeTable->GetNumber("tx", 0.0);
    units::degree_t ty = (units::degree_t) limeTable->GetNumber("ty", 0.0);

    units::meter_t relativeX = (cameraPose.Z() / tan((cameraPose.Rotation().Z().to<double>() - ty.convert<units::angle::radian>().to<double>()))) - cameraPose.Y();
    units::meter_t relativeY = -relativeX * tan(tx.convert<units::angle::radian>().to<double>()) - cameraPose.X();

    units::degree_t robotTheta = estimator->GetEstimatedPosition().Rotation().Degrees(); //Get robot theta from pigeon
    units::degree_t theta = 0_deg;

    if (robotTheta < 0_deg) theta = robotTheta + 360_deg;
    else if (robotTheta > 0_deg) theta = robotTheta - 360_deg;

    units::meter_t currentRobotX = estimator->GetEstimatedPosition().X(); //Get robot X from odom
    units::meter_t currentRobotY = estimator->GetEstimatedPosition().Y(); //Get robot Y from odom

    units::meter_t globalX = units::meter_t(cos(theta.convert<units::degree>().to<double>() * relativeX.to<double>()) - (sin(theta.convert<units::degree>().to<double>() * relativeY.to<double>()))) + currentRobotX;
    units::meter_t globalY = units::meter_t(sin(theta.convert<units::degree>().to<double>() * relativeX.to<double>()) + (cos(theta.convert<units::angle::degree>().to<double>() * relativeY.to<double>()))) + currentRobotY;

    return frc::Pose3d(
        globalX,
        globalY,
        0_m,
        frc::Rotation3d()
    );
}

void GamePieceSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleArrayProperty(
        "gamePiece Pos",
        [this] {
            frc::Pose2d gamePos = getSensor().ToPose2d();
            std::vector<double> gamePosVector{
                gamePos.X().to<double>(),
                gamePos.Y().to<double>(),
                gamePos.Rotation().Degrees().to<double>()
            };
            return gamePosVector;
        },
        nullptr
    );
}
