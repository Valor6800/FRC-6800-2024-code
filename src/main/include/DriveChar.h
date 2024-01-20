#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "frc/DataLogManager.h"

#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/BaseSubsystem.h"

#include "Constants.h"
#include "Drivetrain.h"

class DriveChar : public valor::BaseSubsystem {
 public:
  DriveChar(frc::TimedRobot *_robot, Drivetrain* _drive);

  void assignOutputs() override;

  frc2::CommandPtr GetAutonomousCommand();
  void InitSendable(wpi::SendableBuilder& builder);
  
 private:
  void ConfigureBindings();
  Drivetrain*  drive;
};