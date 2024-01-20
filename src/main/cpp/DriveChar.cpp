#include "DriveChar.h"
#include "Drivetrain.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot, Drivetrain *_drive) : valor::BaseSubsystem(_robot, "Drivetrain"),
  drive(_drive) 
{
  frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void DriveChar::assignOutputs() {
  if (driverGamepad->GetAButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdQuasistatic(frc2::sysid::Direction::kForward);
    cmd.Schedule();
  }
  if (driverGamepad->GetBButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse);
    cmd.Schedule();
  }
  if (driverGamepad->GetXButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdDynamic(frc2::sysid::Direction::kForward);
    cmd.Schedule();
  }
  if (driverGamepad->GetYButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdDynamic(frc2::sysid::Direction::kReverse);
    cmd.Schedule();
  }
}

frc2::CommandPtr DriveChar::GetAutonomousCommand() {
  return drive->Run([] {});
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty("X Pose", [this] {return drive->state.xPose; }, nullptr);
        
}