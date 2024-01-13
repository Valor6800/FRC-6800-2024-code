#pragma once

#include "BaseSensor.h"
#include <frc/geometry/Pose3d.h>
#include "VisionSensor.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include <span>
#include <vector>

namespace valor
{
    class AprilTagsSensor : public BaseSensor<LimeLight> {
        public:
            /**
             * @brief Constructor for AprilTagsSensor
            *
            * @param _robot Pass in the Robot reference so the calculate can be auto-scheduled
            * @param _name The name of the specific sensor for logging and reporting
            */
            AprilTagsSensor(frc::TimedRobot *_robot, const char *_name);

            ~AprilTagsSensor();

            void reset() override;

            void InitSendable(wpi::SendableBuilder& builder) override;

            void adas(int pipe);
        private:
            void calculate() override;

            void setGlobalPosition();
    };
} // namespace valor
