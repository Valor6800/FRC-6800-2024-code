#include "DriveChar.h"
#include "Drivetrain.h"
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot, Drivetrain *_drive) : valor::BaseSubsystem(_robot, "Drivetrain"),
  drive(_drive) 
{
  ConfigureBindings();
}

void DriveChar::ConfigureBindings() {

  while (driverGamepad->GetAButtonPressed()) {drive->SysIdQuasistatic(frc2::sysid::Direction::kForward);}
  while (driverGamepad->GetBButtonPressed()) {drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse);}
  while (driverGamepad->GetXButtonPressed()) {drive->SysIdDynamic(frc2::sysid::Direction::kForward);}
  while (driverGamepad->GetYButtonPressed()) {drive->SysIdDynamic(frc2::sysid::Direction::kReverse);}

}

frc2::CommandPtr DriveChar::GetAutonomousCommand() {
  return drive->Run([] {});
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty("X Pose", [this] {return drive->state.xPose; }, nullptr);
        
}