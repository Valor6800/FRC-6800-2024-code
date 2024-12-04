#ifndef ROBOT_H
#define ROBOT_H

#include "frc/TimedRobot.h"
#include "frc/Joystick.h"
#include <rev/CANSparkMax.h>
#include "rev/CANSparkMax.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/drive/DifferentialDrive.h"
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"

// Robot class declaration
class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;

 private:
  // Motor controllers
  rev::CANSparkMax leftRear;
  rev::CANSparkMax leftFront;
  rev::CANSparkMax rightRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax m_launchWheel;
  rev::CANSparkMax m_feedWheel;
  rev::CANSparkMax m_rollerClaw;
  rev::CANSparkMax m_climber;

  // Joystick controllers
  frc::Joystick m_driverController;
  frc::Joystick m_manipController;

  // Differential drive system
  frc::DifferentialDrive m_drivetrain;

  //time
  frc::Timer m_timer;

  // Constants for current limit
  static constexpr int DRIVE_CURRENT_LIMIT_A = 60;
  static constexpr int FEEDER_CURRENT_LIMIT_A = 60;
  static constexpr int LAUNCHER_CURRENT_LIMIT_A = 60;

  static constexpr double FEEDER_OUT_SPEED = 1.0;
  static constexpr double FEEDER_IN_SPEED = -0.4;
  static constexpr double LAUNCHER_SPEED = 1.0;
  static constexpr double LAUNCHER_AMP_SPEED = 0.17;
  static constexpr double CLAW_OUTPUT_POWER = 0.5;
  static constexpr double CLAW_STALL_POWER = 0.1;
  static constexpr double CLIMBER_OUTPUT_POWER = 1.0;
};

#endif  // ROBOT_H
