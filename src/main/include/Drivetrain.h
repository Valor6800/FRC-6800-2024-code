#pragma once

#include "frc/geometry/Pose3d.h"
#include "units/length.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/Swerve.h"
#include <map>
#include <string>
#include <vector>
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/TimedRobot.h>
#include <frc2/command/FunctionalCommand.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/Pigeon2.hpp>

#define SWERVE_COUNT 4

/**
 * @brief Subsystem - Drivetrain
 * 
 * Subsystem responsible for driving the robot chassis, and housing all the logic to control the
 * 4 swerve modules on the robot.
 */
class Drivetrain : public valor::BaseSubsystem
{
public:

     /**
      * @brief Quick way to select the drive motor controller
      * To change what motor controller runs the drive motor, change this to either:
      * * valor::PhoenixController
      * * valor::NeoController
      */
     typedef valor::NeoController SwerveDriveMotor;

     /**
      * @brief Quick way to select the azimuth motor controller
      * To change what motor controller runs the azimuth motor, change this to either:
      * * valor::PhoenixController
      * * valor::NeoController
      */
     typedef valor::NeoController SwerveAzimuthMotor;

     /**
      * @brief Construct a new Drivetrain object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Drivetrain(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Drivetrain object
      * 
      * Drivetrain objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Drivetrain();

     /**
      * @brief Initialize the drivetrain
      * 
      * Includes:
      * * Calibrating the pigeon
      * * Configuring each swerve module (including controllers for azimuth and drive motors)
      * * Setting the PID values for the Azimuth controller
      * * Resetting the drivetrain state
      */
     void init();
     /**
      * @brief Responsible for setting states to values based on controller inputs. 
     */
     void assessInputs();
     /**
      * @brief Responsible for displaying values to the dashboard and getting values from the dashboard to calculate values. 
     */
     void analyzeDashboard();
     /**
      * @brief Responsible for using states to set power and position to motors and other electronics.
     */
     void assignOutputs();
     /**
      * @brief Resets robot odometry, swerve modules, and drive encoders.
     */
     void resetState();
     /**
      * @brief Calculates and sets the heading lock angle.
     */
     void getSpeakerLockAngleRPS();
     /**
      * @brief Gets the error between the current robot heading and the target robot heading.
      * @returns Returns the error between the current robot heading and the target robot heading in radians. 
     */
     units::radian_t getAngleError();
     /**
      * @brief Gets a radian that is clamped between -1 and 1.
      * @param angle The angle to be clamped.
      * @param max The max possible angle.
      * @returns Returns the clamped angle in radians.
     */
     units::radian_t clampAngleRadianRange(units::radian_t angle, double max);
     /**
      * @brief Responsible for sending values to advantage scope for logging.
     */
     void InitSendable(wpi::SendableBuilder& builder);
     /**!
      * @enum
      * @brief An enum that represents the different limelight pipes for detection.
     */
     enum LimelightPipes{
          APRIL_TAGS,
          TAPE_MID,
          TAPE_HIGH
     };

     struct x
     {
          double xSpeed;
          double ySpeed;
          double rot;

          units::radian_t targetAngle;
          units::angular_velocity::radians_per_second_t angleRPS;
  
          bool adas;
          bool lock;

          LimelightPipes limeLocation;
          
          bool xPose;

          bool isLeveled;
          bool abovePitchThreshold;
          bool isHeadingTrack;

          bool topTape;
          bool bottomTape;

          int stage;

          int trackingID;
          double visionOdomDiff;

          double matchStart;

          frc::Pose2d visionPose;
          frc::Pose2d prevVisionPose;

          units::velocity::meters_per_second_t xSpeedMPS;
          units::velocity::meters_per_second_t ySpeedMPS;
          units::angular_velocity::radians_per_second_t rotRPS;

          frc::Pose2d prevPose;

          units::second_t startTimestamp; // generic
     } state;
     
     
     /**
      * Drive the robot with given x, y and rotational velocities using open loop velocity control
      * @param vx_mps the desired x velocity component in meters per second
      * @param vy_mps the desired y velocity component in meters per second
      * @param omega_radps the desired rotational velocity component in radians per second
      * @param isFOC true if driving field oriented
      */
     void drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC);
     /**
      * @brief Used to drive the robot such that it is not field oriented and is instead robot oriented.
      * @param speeds The speed of the swerve modules.
     */
     void driveRobotRelative(frc::ChassisSpeeds speeds);

     /**
      * @brief Directly set the swerve modules to the specified states
      * @param desiredStates the desired swerve module states
      */
     void setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates);

     /**
      * @brief Used to reset the robot odometry such that the current heading is 0 degrees.
     */
     void resetGyro();
     /**
      * @brief Reset the drive encoders to currently read a position of 0.
      */
     void resetDriveEncoders();
     /**
      * @brief Sets the zero references of the swerve modules.
     */
     void pullSwerveModuleZeroReference();
     /**
      * @brief Gets the rotation of the robot from the pigeon
      * @returns Retuns a Rotation2D representing the rotation of the robot.
     */
     frc::Rotation2d getPigeon();
     /**
      * @brief Gets the rotation of the robot in field-oriented space
      * @returns Returns the rotation of the robot in field-oriented space represented in degrees. 
     */
     units::degree_t getGlobalPitch();

     /**
      * Reset the robot's position on the field. Any accumulted gyro drift will be noted and
      *   accounted for in subsequent calls to getPoseMeters()
      * @param pose The robot's actual position on the field
      */
     void resetOdometry(frc::Pose2d pose);

     /**
      * Get the configured swerve modules
      * @return vector of swerve modules
      */
     std::vector<valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor> *> getSwerveModules();


     /**
      * Returns the current gyro heading of the robot
      * This will be affected by any gyro drift that may have accumulated since last recalibration
      * The angle is continuous from 360 to 361 degrees
      * This allows algorithms that wouldn't want to see a discontinuity in the gyro as it sweeps 
      *   past from 360 to 0 on the second time around.
      * The angle is expected to increase as the gyro turns clockwise
      */
     frc::Rotation2d getHeading(bool);

     //returns angle within the range [-180, 180]
     double angleWrap(double degrees);

     /**
      * Returns the position of the robot on the field in meters
      * @return the pose of the robot (x and y are in meters)
      */
     frc::Pose2d getPose_m();

     /**
      * Returns the kinematics object in use by the swerve drive
      * @return kinematics object
      */
     frc::SwerveDriveKinematics<SWERVE_COUNT>* getKinematics();

     void angleLock();

     frc2::FunctionalCommand* getResetOdom();

     double getDriveMaxSpeed();
     double getAutoMaxSpeed();
     double getAutoMaxAcceleration();
     double getRotationMaxSpeed();
     double getRotationMaxAcceleration();

     void setAutoMaxAcceleration(double acceleration, double multiplier);

     valor::PIDF getXPIDF();
     valor::PIDF getYPIDF();
     valor::PIDF getThetaPIDF();

     frc::TrajectoryConfig & getTrajectoryConfig();

     void setXMode();

     frc2::InstantCommand* getSetXMode();

     void setDriveMotorNeutralMode(valor::NeutralMode mode);

private:
     
     double driveMaxSpeed;
     double rotMaxSpeed;
     double autoMaxSpeed;
     double autoMaxAccel;
     double rotMaxAccel;
     ctre::phoenix6::hardware::Pigeon2 pigeon;
     
     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                           units::velocity::meters_per_second_t,
                                                           units::angular_velocity::radians_per_second_t,
                                                           bool);
     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(frc::ChassisSpeeds chassisSpeeds);
     frc::ChassisSpeeds getRobotRelativeSpeeds();

     void configSwerveModule(int);
     void calculateCarpetPose();

     std::vector<valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor> *> swerveModules;
     std::vector<SwerveAzimuthMotor *> azimuthControllers;
     std::vector<SwerveDriveMotor *> driveControllers;

     wpi::array<frc::Translation2d, SWERVE_COUNT> motorLocations;

     wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> getModuleStates();

     frc::SwerveDriveKinematics<SWERVE_COUNT> * kinematics;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> * estimator;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> * calculatedEstimator;
     frc::Pose2d previousPose;

     frc::TrajectoryConfig * config;

     valor::PIDF xPIDF;
     valor::PIDF yPIDF;
     valor::PIDF thetaPIDF;

     bool swerveNoError;
     
     std::vector<valor::AprilTagsSensor*> aprilTagSensors;

     double doubtX, doubtY, doubtRot;
     units::meter_t visionAcceptanceRadius;
};
