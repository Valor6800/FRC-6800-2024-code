#pragma once

#include <frc/TimedRobot.h>
#include <Drivetrain.h>
#include <unordered_map>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

namespace valor {

class Auto
{
    public:
        Auto();
        void fillAutoList();
        frc2::CommandPtr getCurrentAuto();

        // "Preloads" an auto to be made available the moment its requested through getCurrentAuto,
        // as opposed to waiting to load it when getCurrentAuto is called.
        // Will not re-load if an auto with the provided name is already loaded
        void preloadAuto();
        bool newAutoSelected();

    private:
        frc2::CommandPtr loadedAuto = frc2::cmd::Sequence();
        bool preloadedAuto;
        std::string previousAutoName;
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
};

}
