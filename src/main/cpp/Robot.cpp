#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"

// Constructor initializes the motors and the drivetrain
Robot::Robot()
    : leftRear{1, rev::CANSparkMax::MotorType::kBrushed},
      leftFront{2, rev::CANSparkMax::MotorType::kBrushed},
      rightRear{3, rev::CANSparkMax::MotorType::kBrushed},
      rightFront{4, rev::CANSparkMax::MotorType::kBrushed},
      m_launchWheel{6, rev::CANSparkMax::MotorType::kBrushed},
      m_feedWheel{5, rev::CANSparkMax::MotorType::kBrushed},
      m_climber{7, rev::CANSparkMax::MotorType::kBrushless},
      m_rollerClaw{8, rev::CANSparkMax::MotorType::kBrushed},
      m_driverController{0},
      m_manipController{1},
      m_drivetrain{leftFront, rightFront} {}

void Robot::RobotInit() {
  // Apply current limits to motors
  leftRear.SetSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  leftFront.SetSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  rightRear.SetSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  rightFront.SetSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);

  // Set rear motors to follow front motors
  leftRear.Follow(leftFront);
  rightRear.Follow(rightFront);

  // Invert motors as necessary
  leftFront.SetInverted(true);
  rightFront.SetInverted(false);

  // Set motor idle modes
  m_rollerClaw.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_climber.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Set current limits for launcher and feeder
  m_feedWheel.SetSmartCurrentLimit(FEEDER_CURRENT_LIMIT_A);
  m_launchWheel.SetSmartCurrentLimit(LAUNCHER_CURRENT_LIMIT_A);
  m_feedWheel.SetInverted(true);
  m_launchWheel.SetInverted(true);
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Time (seconds)", static_cast<double>(m_timer.Get()));

}

void Robot::TeleopInit() {
  // Set idle modes to coast during teleop
  leftRear.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rightRear.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::TeleopPeriodic() {
  // Handle launcher control
  if (m_manipController.GetRawButton(1)) {
    m_launchWheel.Set(LAUNCHER_SPEED);
  } else if (m_manipController.GetRawButtonReleased(1)) {
    m_launchWheel.Set(0);
  }

  // Handle feeder control
  if (m_manipController.GetRawButton(6)) {
    m_feedWheel.Set(FEEDER_OUT_SPEED);
  } else if (m_manipController.GetRawButtonReleased(6)) {
    m_feedWheel.Set(0);
  }

  // Handle claw control
  if (m_manipController.GetRawButton(3)) {
    m_rollerClaw.Set(CLAW_OUTPUT_POWER);
  } else if (m_manipController.GetRawButton(4)) {
    m_rollerClaw.Set(-CLAW_OUTPUT_POWER);
  } else {
    m_rollerClaw.Set(0);
  }

  // Handle climber control
  if (m_manipController.GetPOV() == 0) {
    m_climber.Set(CLIMBER_OUTPUT_POWER);
  } else if (m_manipController.GetPOV() == 180) {
    m_climber.Set(-CLIMBER_OUTPUT_POWER);
  } else {
    m_climber.Set(0);
  }

  // Drive control
  m_drivetrain.ArcadeDrive(-m_driverController.GetRawAxis(1), -m_driverController.GetRawAxis(4));
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
