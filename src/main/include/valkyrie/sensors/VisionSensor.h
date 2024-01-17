#pragma once

#include "units/velocity.h"
#include "valkyrie/sensors/BaseSensor.h"
#include <memory>
#include <networktables/NetworkTable.h>
#include <frc/geometry/Pose3d.h>
#include <string>

namespace valor{
struct LimeLight
{
    frc::Pose3d globalPos;
    frc::Pose3d alliancePos;
    std::shared_ptr<nt::NetworkTable> limeTable;
    
    double tx;
    double ty;
    double ta;
    double pipe;
    //std::string pipeType; //Later
};

#define KLIMELIGHT -29.8f // degrees; TODO: Make Modular

class VisionSensor : public valor::BaseSensor<LimeLight> {
    public:

        /**
        * @brief Top level constructor for Base Vision Sensor
        *
        * @param robot Top level robot object to parse out smart dashboard
        * @param name HostName of the Limelight
        * @param cameraPose Physical 3D Position of the Limelights **Lens**
        */
        VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::BaseSensor<LimeLight>(robot, name){cameraPose = _cameraPose;}

        //std::string getPipeType(); //Later

        void init();

        void setPipe(int pipe) {
            currState.limeTable->PutNumber("pipeline", pipe);
        };

        bool hasTarget() {
            return currState.limeTable->GetNumber("tv", 0) > 0;
        }
        
        units::velocity::meters_per_second_t getError(int pipe, double kPLimeLight=0.7) {
            setPipe(pipe);
            if (hasTarget()) {
                double tx = currState.tx;
                double normalizedTx = tx / KLIMELIGHT;
                return units::velocity::meters_per_second_t(((std::fabs(normalizedTx) <= 1 ? normalizedTx : std::copysignf(1.0, normalizedTx) ) * kPLimeLight));
            }
            return units::velocity::meters_per_second_t{0};
        }
    protected:
        frc::Pose3d cameraPose;

        void setDefaultValues() {
            currState.tx = currState.limeTable->GetNumber("tx", 0.0);
            currState.ty = currState.limeTable->GetNumber("ty", 0.0);
            currState.ta = currState.limeTable->GetNumber("ta", 0.0);
            currState.pipe = currState.limeTable->GetNumber("pipeline", 0);
        }
};

}
