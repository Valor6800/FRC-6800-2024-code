#include "DriveChar.h"
#include "Drivetrain.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

DriveChar::DriveChar(frc::TimedRobot *_robot, Drivetrain *_drive) : valor::BaseSubsystem(_robot, "SysID"),
  drive(_drive) 
{
  frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void DriveChar::assignOutputs() {
  if (operatorGamepad->GetAButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdQuasistatic(frc2::sysid::Direction::kForward);
    cmd.Schedule();
  }
  if (operatorGamepad->GetBButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse);
    cmd.Schedule();
  }
  if (operatorGamepad->GetXButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdDynamic(frc2::sysid::Direction::kForward);
    cmd.Schedule();
  }
  if (operatorGamepad->GetYButtonPressed()) {
    frc2::CommandPtr cmd = drive->SysIdDynamic(frc2::sysid::Direction::kReverse);
    cmd.Schedule();
  }
  if (operatorGamepad->GetLeftBumperPressed()) {
    frc2::InstantCommand([this]() {
      table->PutString("hehe", "hi");
    }).Schedule();
  }
}

void DriveChar::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty("X Pose", [this] {return drive->state.xPose; }, nullptr);
        
}