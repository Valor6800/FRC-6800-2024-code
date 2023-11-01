#include "valkyrie/sensors/BaseSensor.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <string>

namespace valor {
class VisionSensor : public valor::BaseSensor<int> {
   public:
     VisionSensor(frc::TimedRobot *_robot, frc::Pose3d _cameraCoordinates);
     void setPipeline(int pipe);
     int getPipeline();
     bool tracking;

     struct Detector {
          struct GamePiece {
               std::pair<int, std::string> piece;
               frc::Pose2d relativeDistance;
               frc::Pose2d globalCoords;
          } currentGamePiece;
     };

     struct AprilTags {
          int trackingID;
          frc::Pose2d visionPosition;
          frc::Pose2d prevVisionPosition;
     };

     struct RetroTape {};

   private:
     std::shared_ptr<nt::NetworkTable> limeTable;
     const frc::Pose3d cameraCoordinate; // relative to the robots center
};
} // namespace valor
