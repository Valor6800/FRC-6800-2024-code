#include "valkyrie/sensors/AprilTagsSensor.h"
#include "frc/DriverStation.h"

#define KLIMELIGHT -29.8f //degrees; TODO: MakeModular
using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot *_robot, const char *_name, frc::Pose3d _cameraPose) :
    VisionSensor(_robot, _name, _cameraPose)
{
    getSensor().limeTable = nt::NetworkTableInstance::GetDefault().GetTable(_name); 
    reset();
}

void AprilTagsSensor::reset() {
    
}

AprilTagsSensor::~AprilTagsSensor() {
    
}

void AprilTagsSensor::calculate() {
    prevState = currState;
    currState = getSensor();

    setDefaultValues();

    setGlobalPosition();
    setAlliancePosition();
}

void AprilTagsSensor::setGlobalPosition() {
    if (!hasTarget()) return;

    std::vector<double> botPose = currState.limeTable->GetNumberArray("botpose", std::span<double>() );
    currState.globalPos = frc::Pose3d(
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

void AprilTagsSensor::setAlliancePosition() {
    if (!hasTarget()) return;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        setBlueAlliancePosition();
    }
    else {
        setRedAlliancePosition();
    }
}

void AprilTagsSensor::setRedAlliancePosition() {
    if (!hasTarget()) return;

    std::vector<double> botPose = currState.limeTable->GetNumberArray("botpose_wpired", std::span<double>() );
    currState.alliancePos = frc::Pose3d(
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

void AprilTagsSensor::setBlueAlliancePosition() {
    if (!hasTarget()) return;

    std::vector<double> botPose = currState.limeTable->GetNumberArray("botpose_wpiblue", std::span<double>() );
    currState.alliancePos = frc::Pose3d(
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

void AprilTagsSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Sensor");
}
