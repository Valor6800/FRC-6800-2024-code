#pragma once

#include <frc/TimedRobot.h>
#include <Drivetrain.h>

namespace valor {

class Auto
{
    public:
        Auto();
        void fillAutoList();
        frc2::CommandPtr getCurrentAuto();
        void scheduleCommands(std::vector<frc2::CommandPtr> commands);
        std::vector<frc2::CommandPtr> makeOTFAuto(std::string autoName);
        std::string getAutoName();

    private:
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
        frc2::CommandPtr getPathFindToPose(frc::Pose2d targetPose, units::meters_per_second_t endVelocity, units::meter_t rotDelay);
        frc2::CommandPtr getFollowPathFind(std::shared_ptr<PathPlannerPath> path, units::meter_t rotDelay);
        std::vector<frc::Pose2d> generatePoses(frc::Pose2d startPose, frc::Pose2d endPose, bool useLimelight);
        std::shared_ptr<PathPlannerPath> makePath(std::vector<frc::Pose2d> poses, units::meters_per_second_t endMPS, units::degree_t endRot);
        frc2::CommandPtr makeCommandFromPath(std::shared_ptr<PathPlannerPath> path);
        bool comparePose2D(frc::Pose2d onePose, frc::Pose2d twoPose);
        frc::Pose2d getEndPoseFromPath(std::shared_ptr<PathPlannerPath> path);
        frc::Pose2d getOTFEndPose(bool limelight, units::meter_t X, units::meter_t Y, units::degree_t rot);
};

}