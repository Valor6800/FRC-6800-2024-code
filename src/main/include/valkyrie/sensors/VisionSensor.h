#include "valkyrie/sensors/BaseSensor.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <string>
#include <unordered_map>

namespace valor {

class VisionSensor : public valor::BaseSensor<int> {

  public:
    VisionSensor(frc::TimedRobot *_robot, const char *_name,
                 frc::Pose3d _cameraCoordinates);

    void setPipeline(int pipe);
    struct Pipeline {};
    struct {
        int pipe;
        bool tracking;
        Pipeline currentPipe;
    } state;

    VisionSensor(frc::TimedRobot *_robot, const char *_name,
                 frc::Pose3d _cameraCoordinates,
                 std::unordered_map<int, PipeLines> myPipes);

  protected:
    std::shared_ptr<nt::NetworkTable> limeTable;

  private:
    frc::Pose3d cameraCoordinate; // relative to the robots center
    std::unordered_map<int, PipeLines> myPipes;
};

} // namespace valor
