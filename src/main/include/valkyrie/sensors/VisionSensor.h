#pragma once

#include "networktables/NetworkTableInstance.h"
#include "units/velocity.h"
#include "valkyrie/sensors/BaseSensor.h"
#include <functional>
#include <memory>
#include <networktables/NetworkTable.h>
#include <frc/geometry/Pose3d.h>

namespace valor{

#define KLIMELIGHT -29.8f // degrees; TODO: Make Modular

class VisionSensor : public valor::BaseSensor<frc::Pose3d> {
    public:

        /**
        * @brief Top level constructor for Base Vision Sensor
        *
        * @param robot Top level robot object to parse out smart dashboard
        * @param name HostName of the Limelight
        * @param cameraPose Physical 3D Position of the Limelights **Lens**
        */
        VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::BaseSensor<frc::Pose3d>(robot, name){
            cameraPose = _cameraPose;
            limeTable = nt::NetworkTableInstance::GetDefault().GetTable(name);
            setGetter([this](){return getGlobalPose();});
        }

        //std::string getPipeType(); //Later

        virtual frc::Pose3d getGlobalPose() = 0;

        void init();

        void setPipe(int _pipe) {
            if (limeTable == nullptr) return;
            limeTable->PutNumber("pipeline", _pipe);
        };

        bool hasTarget() {
            return limeTable != nullptr && tv == 1;
        }
        
        units::velocity::meters_per_second_t getError(int pipe, double kPLimeLight=0.7) {
            if (limeTable == nullptr) return units::velocity::meters_per_second_t{0};
            setPipe(pipe);
            if (hasTarget()) {
                double normalizedTx = tx / KLIMELIGHT;
                return units::velocity::meters_per_second_t(((std::fabs(normalizedTx) <= 1 ? normalizedTx : std::copysignf(1.0, normalizedTx) ) * kPLimeLight));
            }
            return units::velocity::meters_per_second_t{0};
        }

        frc::Pose3d cameraPose;
        std::shared_ptr<nt::NetworkTable> limeTable;

        virtual void InitSendable(wpi::SendableBuilder& builder) = 0;
    protected:
        double tx, ty, tv;
        int pipe;

        void setDefaultValues() {
            if (!hasTarget() || limeTable == nullptr) {
                tv = 0;
                tx = 0;
                ty = 0;
                pipe = 0;
                return;
            }

            tx = limeTable->GetNumber("tx", 0.0);
            ty = limeTable->GetNumber("ty", 0.0);
            tv = limeTable->GetNumber("tv", 0.0);
            pipe = limeTable->GetNumber("pipeline", 0);
        }

    private:
        void calculate() override {
            prevState = currState;
            setDefaultValues();
            currState = getSensor();
        }
};

}
