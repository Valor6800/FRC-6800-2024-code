#include <frc2/command/sysid/SysIdRoutine.h>
#include "DriveChar.h"

#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Drivetrain") {
  ConfigureBindings();
}

void SysIdRoutineBot::ConfigureBindings() {
  while (driverGamepad->GetAButtonPressed()){
    frc2::sysid::SysIdRoutine::Quasistatic(frc2::sysid::Direction::kForward);
  }
  while (driverGamepad->GetBButtonPressed()){
    m_drive.Quasistatic(frc2::sysid::Direction::kReverse);
  }
  while (driverGamepad->GetXButtonPressed()){
    m_drive.Dynamic(frc2::sysid::Direction::kForward);
  }
  while (driverGamepad->GetYButtonPressed()){
    m_drive.Dynamic(frc2::sysid::Direction::kReverse);
  }
}

frc2::CommandPtr SysIdRoutineBot::GetAutonomousCommand() {
  return m_drive.Run([] {});
}