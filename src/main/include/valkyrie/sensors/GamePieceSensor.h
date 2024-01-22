#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "valkyrie/sensors/VisionSensor.h"

namespace valor {


class GamePieceSensor : public valor::VisionSensor {
    public:
        GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator);
           
        void InitSendable(wpi::SendableBuilder& builder) override;
        
    private:
        frc::Pose3d getGlobalPose() override;
        frc::SwerveDrivePoseEstimator<4>* estimator;
};
}
