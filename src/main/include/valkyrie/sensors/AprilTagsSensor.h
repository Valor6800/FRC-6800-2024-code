#pragma once

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
            AprilTagsSensor(frc::TimedRobot *_robot, const char *_name, frc::Pose3d _cameraPose);

            ~AprilTagsSensor();

            void reset() override;

            void InitSendable(wpi::SendableBuilder& builder) override;

        private:
            enum RobotTransform {
                GLOBAL = 0,
                BLUE_ALLIANCE = 1,
                RED_ALLIANCE = 2,
                /*CAMERA_TARGET = 3,
                TARGET_CAMERA = 4,
                TARGET_ROBOT = 5,
                ROBOT_TARGET = 6,
                CAMERA_ROBOT = 7*/
            };

            void calculate() override;

            void setGlobalPosition();
            void setAlliancePosition();
            void setRedAlliancePosition();
            void setBlueAlliancePosition();
    };
} // namespace valor
