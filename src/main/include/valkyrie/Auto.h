// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include <unordered_map>

#include <Drivetrain.h>

namespace valor {

class Auto {
   public:
    Auto();

    /**
     * Fills the SendableChooser with a list of .auto filenames found in ~/deploy/pathplanner/autos key'd with a
     * corresponding "human readable" name.
     */
    void fillAutoList();

    /**
     * Loads a selected pathplanner auto or returns the preloaded auto.
     *
     * @param filename Filename of the pathplanner auto to get
     * @returns The generated auto
     */
    frc2::CommandPtr getAuto(std::string filename);

    /**
     * Loads the auto with the filename selected in the SendableChooser.
     *
     * @returns The generated auto.
     */
    frc2::CommandPtr getSelectedAuto();

    /**
     * Loads and stores an auto to be retrieved with getAuto. This cuts out the delay that would normally happen if the
     * auto was generated on AutonomousInit.
     *
     * Does nothing if an auto with the given filename is already loaded. Use clearAutos to clear the preloaded autos
     * list, then rebuild them.
     *
     * @param filename Filename of the pathplanner auto to preload
     */
    void preloadAuto(std::string filename);

    /**
     * Preloads the auto selected in the SendableChooser.
     */
    void preloadSelectedAuto();

    /**
     * Clears all preloaded autos.
     */
    void clearAutos();

   private:
    std::vector<std::pair<std::string, frc2::CommandPtr>> loadedAutos;
    frc2::CommandPtr makeAuto(std::string autoName);
    std::shared_ptr<nt::NetworkTable> table;
    frc::SendableChooser<std::string> m_chooser;
};

}  // namespace valor
