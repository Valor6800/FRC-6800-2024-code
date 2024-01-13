#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <memory>
#include <networktables/NetworkTable.h>
#include <frc/geometry/Pose3d.h>
#include <string>

namespace valor{
struct LimeLight
{
    frc::Pose3d globalPose;
    std::shared_ptr<nt::NetworkTable> limeTable;
    
    double tx;
    double ty;
    double ta;
    double pipe;
};

class VisionSensor : public valor::BaseSensor<LimeLight> {
    public:
        /**
         * @param robot TimedRobot
        */
        VisionSensor(frc::TimedRobot* robot, frc::Pose3d cameraPose);
       
        std::string getPipeType();

        void init();
};

}
