#pragma once

#include "networktables/NetworkTableInstance.h"
#include "units/velocity.h"
#include "valkyrie/sensors/BaseSensor.h"
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
        VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose);
        //std::string getPipeType(); //Later

        virtual frc::Pose3d getGlobalPose() = 0;

        void reset() override;

        void setPipe(int _pipe);

        bool hasTarget();
        
        units::velocity::meters_per_second_t getError(int pipe, double kPLimeLight=0.7);

        virtual void InitSendable(wpi::SendableBuilder& builder) override = 0;

    protected:
        double tx, ty, tv;
        int pipe;

        void setDefaultValues();

        frc::Pose3d cameraPose;
        std::shared_ptr<nt::NetworkTable> limeTable;

    private:
        void calculate() override;
};

}
