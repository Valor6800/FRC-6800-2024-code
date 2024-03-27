#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"

namespace valor {


class GamePieceSensor : public valor::VisionSensor {
    public:
        GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator);
           
        void InitSendable(wpi::SendableBuilder& builder) override;
        
        struct {units::meter_t x = 0_m; units::meter_t y = 0_m;} relativePose;
        
    private:

        frc::Pose3d getGlobalPose() override;
        void updateRelative();

        frc::SwerveDrivePoseEstimator<4>* estimator;
};
}
