#pragma once

#include <frc/TimedRobot.h>
#include <Drivetrain.h>
#include <unordered_map>

namespace valor {

class Auto
{
    public:
        Auto();
        void fillAutoList();
        std::string getSelectedAutoName();
        frc2::CommandPtr getCurrentAuto();

        // "Preloads" an auto to be made available the moment its requested through getCurrentAuto,
        // as opposed to waiting to load it when getCurrentAuto is called.
        // Will not re-load if an auto with the provided name is already loaded
        void preloadAuto(std::string autoName);

    private:
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
        std::unordered_map<std::string, frc2::CommandPtr> loadedAutos;
};

}
