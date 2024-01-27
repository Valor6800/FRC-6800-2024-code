#pragma once

#include "frc/estimator/PoseEstimator.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose3d.h"
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

            void InitSendable(wpi::SendableBuilder& builder) override;

            void applyVisionMeasurement(frc::SwerveDrivePoseEstimator<4> *estimator, double outlier, double doubtX = 1, double doubtY = 1, double doubtRot = 1);

        private:
            frc::Pose3d getGlobalPose() override;
            
    };
} // namespace valor
