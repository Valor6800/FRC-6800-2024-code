#include "valkyrie/sensors/GamePieceSensor.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "units/length.h"
#include "wpi/detail/value_t.h"
#include <cmath>
#include <vector>
using namespace valor;

GamePieceSensor::GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator) : valor::VisionSensor(robot, name, _cameraPose),
    estimator(estimator)
{
    setGetter([this](){return getGlobalPose();});
}

frc::Pose3d GamePieceSensor::getGlobalPose() {

    updateRelative();

    if (!hasTarget() || estimator->GetEstimatedPosition().X() == 0.0_m || estimator->GetEstimatedPosition().Y() == 0.0_m) return frc::Pose3d();
    

    units::degree_t robotTheta = estimator->GetEstimatedPosition().Rotation().Degrees(); //Get robot theta from pigeon
    units::degree_t theta = 0_deg;

    if (robotTheta < 0_deg) theta = robotTheta + 360_deg;
    else if (robotTheta > 0_deg) theta = robotTheta - 360_deg;

    units::meter_t currentRobotX = estimator->GetEstimatedPosition().X(); //Get robot X from odom
    units::meter_t currentRobotY = estimator->GetEstimatedPosition().Y(); //Get robot Y from odom

    units::degree_t t1 = theta.convert<units::degree>() * relativePose.x.to<double>();
    units::degree_t t2 = theta.convert<units::degree>() * relativePose.y.to<double>();

    units::meter_t globalX = units::meter_t(cos(t1.convert<units::radian>().to<double>()) - sin(t2.convert<units::radian>().to<double>())) + currentRobotX;
    units::meter_t globalY = units::meter_t(sin(t1.convert<units::radian>().to<double>()) + cos(t2.convert<units::radian>().to<double>())) + currentRobotY;

    return frc::Pose3d(
       globalX,
       globalY,
       0_m,
       frc::Rotation3d()
    );
}

void GamePieceSensor::updateRelative() {
    if (!hasTarget()) return;
    units::degree_t ty = (units::degree_t) limeTable->GetNumber("ty", 0.0);
    units::degree_t tx = (units::degree_t) limeTable->GetNumber("tx", 0.0);

    relativePose.x = cameraPose.Z() * tanf((M_PI/2.0) + cameraPose.Rotation().Y().to<double>() + ty.convert<units::angle::radian>().to<double>());
    relativePose.y = relativePose.x / (tanf((M_PI/2.0) - cameraPose.Rotation().Z().to<double>() + tx.convert<units::angle::radian>().to<double>()));
}

void GamePieceSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleArrayProperty(
        "gamePiece Pos",
        [this] {
            frc::Pose2d gamePos = getSensor().ToPose2d();
            std::vector<double> gamePosVector{
                gamePos.Y().to<double>(),
                gamePos.X().to<double>(),
            };
            return gamePosVector;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Relative Pos",
        [this]
        {
            return std::vector<double>{
                relativePose.x.to<double>(), // Fd
                relativePose.y.to<double>() // lt and rt
            };
        },
        nullptr
    );
}
