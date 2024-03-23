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
#include <cstdint>
#include <unordered_map>

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

            struct TagData {
                double id, txnc, tync, ta;
                units::meter_t distanceToCamera, distanceToRobot;
                double ambiguity;
            };

            int getTagID();

            void InitSendable(wpi::SendableBuilder& builder) override;

            units::meter_t normalVisionOutlier = 4.5_m;
            void applyVisionMeasurement(frc::SwerveDrivePoseEstimator<4> *estimator, units::velocity::meters_per_second_t speed, bool accept = true, double doubtX = 1, double doubtY = 1, double doubtRot = 1);

            frc::Pose3d getPoseFromAprilTag();

            TagData getTagData(int i);
        private:
            frc::Pose3d getGlobalPose() override;

            double dp, vp;
            units::meter_t distance{0_m};
            int tagCount;

            bool tagMapUpdate;

            
            
            std::unordered_map<int, TagData> tagsSeen;

            bool updateTagMap(std::vector<double> data);
    
            std::vector<double> botPose;
            std::vector<double> botToTargetPose;
    };
} // namespace valor
