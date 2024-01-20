#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/sysid/SysIdRoutine.h>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/BaseSubsystem.h"

#include "Constants.h"
#include "Drivetrain.h"

class SysIdRoutineBot : public valor::BaseSubsystem {
 public:
  SysIdRoutineBot(frc::TimedRobot *robot);

  frc2::CommandPtr GetAutonomousCommand();
  void InitSendable(wpi::SendableBuilder& builder);
  
 private:
  void ConfigureBindings();
  frc2::sysid::SysIdRoutine sysIdRoutine;
  Drivetrain m_drive;
};