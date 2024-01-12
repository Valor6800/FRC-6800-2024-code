#pragma once

#include <frc/TimedRobot.h>
#include <Drivetrain.h>

class Auto
{
    public:
        Auto(Drivetrain *drivetrain);
        
        void fillAutoList();
        frc2::CommandPtr getCurrentAuto();

    private:
        Drivetrain *drivetrain;
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
};