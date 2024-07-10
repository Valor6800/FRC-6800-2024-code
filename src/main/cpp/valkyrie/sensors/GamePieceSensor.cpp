#include "valkyrie/sensors/GamePieceSensor.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "units/length.h"
#include "wpi/detail/value_t.h"
#include <cmath>
#include <vector>
// clang-format off
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

    units::degree_t t1 = theta.convert<units::degree>() * relativePoseFromCamera.x.to<double>();
    units::degree_t t2 = theta.convert<units::degree>() * relativePoseFromCamera.y.to<double>();

    units::meter_t globalX = units::meter_t(cos(t1.convert<units::radian>().to<double>()) - sin(t2.convert<units::radian>().to<double>())) + currentRobotX;
    units::meter_t globalY = units::meter_t(sin(t1.convert<units::radian>().to<double>()) + cos(t2.convert<units::radian>().to<double>())) + currentRobotY;

    return frc::Pose3d(
       globalX,
       globalY,
       0_m,
       frc::Rotation3d()
    );
}
// clang-format on
void GamePieceSensor::updateRelative() {
    if (!hasTarget()) return;

    relativePoseFromCamera.x = cameraPose.Z() * tanf((M_PI/2.0) + cameraPose.Rotation().Y().to<double>() + ty.convert<units::angle::radian>().to<double>());
    relativePoseFromCamera.y = relativePoseFromCamera.x / (tanf((M_PI/2.0) - cameraPose.Rotation().Z().to<double>() + tx.convert<units::angle::radian>().to<double>()));

    updateRelativeToCenter();
}

void GamePieceSensor::updateRelativeToCenter() {
    units::meter_t cameraToGamePiece{
        sqrtf(powf(relativePoseFromCamera.x.to<double>(), 2) +
              powf(relativePoseFromCamera.y.to<double>(), 2))
    }; 

    units::meter_t robotToCamera{
        sqrtf(powf(cameraPose.X().to<double>(), 2) +
              powf(cameraPose.Y().to<double>(), 2))
    };

    units::radian_t camYaw = cameraPose.Rotation().Y();
    units::radian_t alpha{ atan2f(cameraPose.Y().to<double>(), cameraPose.X().to<double>()) };

    double a {cameraToGamePiece.to<double>()};
    double b {robotToCamera.to<double>()};
    double gcr {
        camYaw.to<double>() - ty.convert<units::radians>().to<double>() + (M_PI / 2) - alpha.to<double>()
    };

    units::meter_t robotToGamePiece{
        sqrtf(powf(a, 2) + powf(b, 2) - (2 * a * b * cosf(gcr)))
    };

    units::radian_t grc {
        asinf((a * sinf(gcr)) / robotToGamePiece.to<double>())
    };

    relativePoseFromCenter.x = 
        sinf((((units::radian_t)((M_PI / 2) - alpha.to<double>())) + grc).to<double>()) * robotToGamePiece;

    relativePoseFromCenter.y = 
        cosf((((units::radian_t)((M_PI / 2) - alpha.to<double>())) + grc).to<double>()) * robotToGamePiece;

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
                relativePoseFromCamera.x.to<double>(), // Fd
                relativePoseFromCamera.y.to<double>() // lt and rt
            };
        },
        nullptr
    );
}
