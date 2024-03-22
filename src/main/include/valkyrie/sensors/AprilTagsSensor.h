#pragma once

#include "frc/estimator/PoseEstimator.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose3d.h"
#include "units/length.h"
#include "units/velocity.h"
#include "valkyrie/sensors/VisionSensor.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace valor
{
    class AprilTagsSensor : public VisionSensor {
        public:
            /**
             * @brief Constructor for AprilTagsSensor
            *
            * @param _robot Pass in the Robot reference so the calculate can be auto-scheduled
            * @param _name The name of the specific sensor for logging and reporting
            */
            AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose);

            int getTagID();

            void InitSendable(wpi::SendableBuilder& builder) override;

            units::meter_t normalVisionOutlier = 4.5_m;
            void applyVisionMeasurement(frc::SwerveDrivePoseEstimator<4> *estimator, units::velocity::meters_per_second_t speed, bool accept = true, double doubtX = 1, double doubtY = 1, double doubtRot = 1);

            frc::Pose3d getPoseFromAprilTag();

            units::meter_t getDistanceToRobot();
            units::meter_t getDistanceToCamera();

            double getTagCount();
            double getTagAmbiguity();

        private:
            frc::Pose3d getGlobalPose() override;
            units::meter_t distance{0_m};
            double dp, vp;
            double ambiguity, tagCount;
            units::meter_t distanceToCamera, distanceToRobot;

            void updateMiscValues();
    
            std::vector<double> botPose;
            std::vector<double> botToTargetPose;
    };
} // namespace valor
