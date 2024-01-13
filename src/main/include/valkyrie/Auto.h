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
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
};

}