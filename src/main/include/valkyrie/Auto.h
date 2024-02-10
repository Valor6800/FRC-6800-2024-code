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

    private:
        std::vector<frc::Translation2d> getBezierPoints(std::vector<frc::Pose2d> poses);
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
        std::vector<frc2::CommandPtr> pathCommands;
        frc2::CommandPtr getPathFindToPose(frc::Pose2d targetPose, units::meters_per_second_t endVelocity, units::meter_t rotDelay);
};

}