#pragma once

#include "BaseSensor.h"
#include <frc/geometry/Pose3d.h>
#include "VisionSensor.h"

namespace valor
{
    class AprilTagsSensor : public BaseSensor<LimeLight> {
        public:
            /**
             * @brief Constructor for AprilTagsSensor
            */
            AprilTagsSensor(frc::TimedRobot *_robot, const char *_name);

            void reset();

        private:
            void calculate();

            void setGlobalPosition();

    };
} // namespace valor
