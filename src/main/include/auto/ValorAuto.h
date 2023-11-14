#pragma once

#include "Drivetrain.h"
#include "Intake.h"
#include "Elevarm.h"

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>

class ValorAuto {
    public: 
        ValorAuto(Drivetrain*, Intake*, Elevarm*);
        ~ValorAuto();

        void fillAutoList();
        frc2::CommandPtr getCurrentAuto();

    private:
        std::unordered_map<std::string, frc2::Command*> compiledCommands;
        Drivetrain *drivetrain;
        Intake *intake;
        Elevarm *elevarm;
        frc::SendableChooser<std::string> m_chooser;
        frc2::CommandPtr makeAuto(std::string);

        std::unordered_map<std::string, std::shared_ptr<frc2::Command> > eventMap;
        // std::unordered_map<std::string, frc2::Command*> testAutos;
        frc2::Command* createPPTrajectoryCommand(pathplanner::PathPlannerTrajectory);
        std::vector<frc2::CommandPtr> trajectoryCommands;
        frc2::CommandPtr driveCommand = frc2::cmd::Sequence();

        std::shared_ptr<nt::NetworkTable> table;

        // Verify compilability of passed in autos
        // Returns: list of broken autos
        // std::vector<std::string> verifyAutos(std::vector<std::string>);
};
