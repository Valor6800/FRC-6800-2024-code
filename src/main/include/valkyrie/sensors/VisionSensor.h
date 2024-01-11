#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <networktables/NetworkTable.h>
#include <frc/geometry/Pose3d.h>

namespace valor{
struct LimeLight
{
    frc::Pose3d globalPose;
};

class VisionSensor : public valor::BaseSensor<int> {
    public:
    /**
     * @param robot TimedRobot
    */
        VisionSensor(frc::TimedRobot* robot, std::shared_ptr<nt::NetworkTable> limeTable, frc::Pose3d cameraPose);
        
        void init();
    protected:
        std::shared_ptr<nt::NetworkTable> limeTable;
        frc::Pose3d cameraPose;
};

}