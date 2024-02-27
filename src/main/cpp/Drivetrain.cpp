#include "Drivetrain.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <string>
#include "Constants.h"
#include "units/length.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"

using namespace pathplanner;

#define SPEAKER_Y 5.543042_m
#define SPEAKER_BLUE_X 0.0_m
#define SPEAKER_RED_X 16.4846_m

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
// #define KP_LOCK 0.2f
#define KP_LIMELIGHT 0.7f

#define KPX 60.0f //50
#define KIX 0.0f //0
#define KDX 0.0f //.1

#define KPY 60.0f //65
#define KIY 0.0f //0
#define KDY 0.0f //.1

#define KPT 15.0f
#define KIT 0.0f
#define KDT 0.0f

#define WHEEL_DIAMETER_M 0.0973f //0.1016
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define ROT_SPEED_MUL 2.0f

#define AUTO_VISION_THRESHOLD 4.0f //meters

#define X_TIME 214.85f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS ""
#define PIGEON_CAN_BUS "baseCAN"

#define KP_ROTATE -0.6f
#define SPEAKER_X_OFFSET 0.15f
#define SPEAKER_Y_OFFSET 0.00f

#define VISION_ACCEPTANCE 4.0_m // meters

#define BLUE_AMP_ROT_ANGLE -90.0_deg
#define BLUE_SOURCE_ROT_ANGLE -30_deg
#define RED_SOURCE_ROT_ANGLE -150_deg
#define BLUE_LOCK_ANGLE 0_deg
#define RED_LOCK_ANGLE 180_deg

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Drivetrain"),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        calculatedEstimator(NULL),
                        swerveNoError(true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        delete azimuthControllers[i];
        delete driveControllers[i];
        delete swerveModules[i];
    }

    delete estimator;
    delete calculatedEstimator;
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{Constants::moduleDiff() * MDX[i],
                                           Constants::moduleDiff() * MDY[i]};

    valor::PIDF azimuthPID;
    azimuthPID.maxVelocity = Constants::azimuthKVel();
    azimuthPID.maxAcceleration = Constants::azimuthKAcc();
    azimuthPID.P = Constants::azimuthKP();
    azimuthPID.error = 0.0027;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      valor::NeutralMode::Brake,
                                                      Constants::swerveAzimuthsReversals()[i],
                                                      1.0 / AZIMUTH_GEAR_RATIO,
                                                      azimuthPID,
                                                      12.0,
                                                      PIGEON_CAN_BUS));
    
    azimuthControllers[i]->setupCANCoder(CANIDs::CANCODER_CANS[i], Constants::swerveZeros()[i], 1.0, false, PIGEON_CAN_BUS);

    valor::PIDF drivePID;
    drivePID.maxVelocity = Constants::driveKVel();
    drivePID.maxAcceleration = Constants::driveKAcc();
    drivePID.P = Constants::driveKP();
    drivePID.error = 0.0027;

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    valor::NeutralMode::Coast,
                                                    Constants::swerveDrivesReversals()[i],
                                                    DRIVETRAIN_CAN_BUS));
    double conversion = 1.0 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M;
    driveControllers[i]->setConversion(conversion);
    driveControllers[i]->setPIDF(drivePID, 0);
    driveControllers[i]->setMaxCurrent(40);

    driveMaxSpeed = driveControllers[i]->getMaxMotorSpeed() / 60.0 * conversion;

    swerveModules.push_back(new valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);

}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{

    for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::aprilCameras) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));
    }

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        aprilLime->setPipe(valor::VisionSensor::PIPELINE_0);
    }

    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        configSwerveModule(i);
    }

    pigeon.GetConfigurator().Apply(
        ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose(
            ctre::phoenix6::configs::MountPoseConfigs{}
            .WithMountPosePitch(Constants::pigeonMountPitch().to<double>())
            .WithMountPoseRoll(Constants::pigeonMountRoll().to<double>())
            .WithMountPoseYaw(Constants::pigeonMountYaw().to<double>())
        )
    );

    kinematics = new frc::SwerveDriveKinematics<SWERVE_COUNT>(motorLocations);
    estimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});
    calculatedEstimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    yPIDF.P = KPY;
    yPIDF.I = KIY;
    yPIDF.D = KDY;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    table->PutNumber("Vision Std", 3.0);

    table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );
    table->PutNumber("DoubtX", 1.0);
    table->PutNumber("DoubtY", 1.0);
    table->PutNumber("DoubtRot", 1.0);

    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    table->PutNumber("KP_ROTATION", KP_ROTATE);
    table->PutNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET);
    table->PutNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);


    state.lock = false;

    resetState();

    AutoBuilder::configureHolonomic(
        [this](){ return getPose_m(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(xPIDF.P, xPIDF.I, xPIDF.D), // Translation PID constants
            PIDConstants(thetaPIDF.P, thetaPIDF.I, thetaPIDF.D), // Rotation PID constants
            units::meters_per_second_t{driveMaxSpeed}, // Max module speed, in m/s
            Constants::driveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

std::vector<valor::Swerve<Drivetrain::SwerveAzimuthMotor, Drivetrain::SwerveDriveMotor> *> Drivetrain::getSwerveModules()
{
    return swerveModules;
}

double Drivetrain::angleWrap(double degrees)
{
    double zeroTo360 = degrees + 180;
    double start = fmod(zeroTo360, 360); //will work for positive angles

    //angle is (-360, 0), add 360 to make (0, 360)
    if (start < 0)
    {
        start += 360;
    }

    //bring it back to (-180, 180)
    return start - 180;
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected())
        return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }

    state.ampAlign = driverGamepad->GetBButton();
    state.isHeadingTrack = driverGamepad->leftTriggerActive();
    state.trapAlign = driverGamepad->GetXButton();
    state.sourceAlign = driverGamepad->GetYButton();
    state.thetaLock = driverGamepad->GetAButton();

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);

    state.rot = driverGamepad->rightStickX(3);

    if (!operatorGamepad || !operatorGamepad->IsConnected())
        return;

    state.topTape = operatorGamepad->DPadUp();
    state.bottomTape = operatorGamepad->DPadRight();

}

void Drivetrain::calculateCarpetPose()
{
    calculatedEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    frc::Pose2d newPose = calculatedEstimator->GetEstimatedPosition();
    units::meter_t deltaX = newPose.X() - previousPose.X();
    double factor = 1.0;
    if (Constants::roughTowardsRedAllianceWall) {
        factor = deltaX < units::meter_t{0} ? Constants::carpetGrainMultipler : 1;
    } else {
        factor = deltaX > units::meter_t{0} ? Constants::carpetGrainMultipler : 1;
    }
    
    calculatedEstimator->AddVisionMeasurement(
        frc::Pose2d{
            factor * deltaX + previousPose.X(),
            newPose.Y(),
            newPose.Rotation()
        },
        frc::Timer::GetFPGATimestamp(),
        {0.1, 0.1, 0.1}
    );
    previousPose = calculatedEstimator->GetEstimatedPosition();
}

void Drivetrain::analyzeDashboard()
{
    if(state.isHeadingTrack) getSpeakerLockAngleRPS();
    else if(state.sourceAlign) setAlignmentAngle(Alignment::SOURCE);
    else if(state.ampAlign) setAlignmentAngle(Alignment::AMP);
    else if(state.trapAlign) setAlignmentAngle(Alignment::TRAP);
    else if(state.thetaLock) setAlignmentAngle(Alignment::LOCK);

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    calculateCarpetPose();

    frc::Pose2d botpose;
    
    doubtX = table->GetNumber("DoubtX", 1.0);
    doubtY = table->GetNumber("DoubtY", 1.0);
    doubtRot = table->GetNumber("DoubtRot", 1.0);
    visionAcceptanceRadius = (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>());

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        aprilLime->applyVisionMeasurement(calculatedEstimator, visionAcceptanceRadius, doubtX, doubtY);
    }

    if (driverGamepad && driverGamepad->IsConnected() && driverGamepad->GetStartButton()) {
        for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
            if (aprilLime->hasTarget()) {
                botpose = aprilLime->getSensor().ToPose2d();
                resetOdometry(botpose);
                break;
            }
        }
    }

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        state.distanceFromSpeaker = units::meter_t(sqrtf(powf((calculatedEstimator->GetEstimatedPosition().X() - SPEAKER_BLUE_X).to<double>(), 2) + powf((calculatedEstimator->GetEstimatedPosition().Y() - SPEAKER_Y).to<double>(), 2)));
    }
    else{
        state.distanceFromSpeaker = units::meter_t(sqrtf(powf((calculatedEstimator->GetEstimatedPosition().X() - SPEAKER_RED_X).to<double>(), 2) + powf((calculatedEstimator->GetEstimatedPosition().Y() - SPEAKER_Y).to<double>(), 2)));
    }
}

void Drivetrain::assignOutputs()
{
    double kPRot = table->GetNumber("KP_ROTATION", KP_ROTATE);
    state.angleRPS = units::angular_velocity::radians_per_second_t{getAngleError().to<double>()*kPRot*rotMaxSpeed};
    state.xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    state.ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
        state.xSpeedMPS *= -1.0;
        state.ySpeedMPS *= -1.0;
    }
    state.rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};

    if(state.ampAlign || state.trapAlign || state.sourceAlign || state.isHeadingTrack || state.thetaLock){
        drive(state.xSpeedMPS, state.ySpeedMPS, state.angleRPS, true);
    } 
    else {
        setDriveMotorNeutralMode(valor::NeutralMode::Coast);
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    }
}

void Drivetrain::getSpeakerLockAngleRPS(){
    units::radian_t targetRotAngle;
    units::meter_t roboXPos = calculatedEstimator->GetEstimatedPosition().X();
    units::meter_t roboYPos = calculatedEstimator->GetEstimatedPosition().Y();

    double speakerX = (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? SPEAKER_RED_X : SPEAKER_BLUE_X).to<double>();
    double redMultiplier = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? -1.0 : 1.0;
    double speakerXOffset = table->GetNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET) * redMultiplier;
    double speakerYOffset = table->GetNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);
    state.targetAngle = units::radian_t(atan2(
        (roboYPos.to<double>() - (SPEAKER_Y.to<double>() + speakerYOffset)),
        (roboXPos.to<double>() - (speakerX + speakerXOffset))
    ));
}

units::radian_t Drivetrain::getAngleError(){
    units::radian_t robotRotation = calculatedEstimator->GetEstimatedPosition().Rotation().Radians();
    double errorAngle = robotRotation.to<double>() - state.targetAngle.to<double>();
    if(errorAngle > PI) {
        return units::radian_t(errorAngle - 2*PI);
    }
    else if(errorAngle < -PI){
        return units::radian_t(2*PI + errorAngle);
    }
    else{
        return (robotRotation - state.targetAngle);
    }
}

void Drivetrain::setAlignmentAngle(Alignment align) {
    bool isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

    if (align == Alignment::AMP) state.targetAngle = BLUE_AMP_ROT_ANGLE;
    else if (align == Alignment::TRAP){
        for(valor::AprilTagsSensor* aprilLime :  aprilTagSensors){
            if(aprilLime->getTagID() == 16){
                // state.targetAngle = units::radian_t(BLUE_RIGHT_TRAP_ROT_ANGLE);
            }
            else if(aprilLime->getTagID() == 15){
                // state.targetAngle = units::radian_t(BLUE_LEFT_TRAP_ROT_ANGLE);
            }
            else if(aprilLime->getTagID() == 14){
                // state.targetAngle = units::radian_t(BLUE_CENTER_TRAP_ROT_ANGLE);
            }
        }
        state.targetAngle = 0_deg;
    }
    else if (align == Alignment::SOURCE)
        state.targetAngle = isRed ? RED_SOURCE_ROT_ANGLE : BLUE_SOURCE_ROT_ANGLE;
    else 
        state.targetAngle = isRed ? RED_LOCK_ANGLE : BLUE_LOCK_ANGLE;
}

double Drivetrain::clampAngleRadianRange(units::radian_t angle, double max){
    return ((angle)/(max)).to<double>();
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> Drivetrain::getModuleStates()
{
    wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, SWERVE_COUNT>(wpi::empty_array);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }
    return modulePositions;
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    estimator->ResetPosition(getPigeon(), getModuleStates(), pose);
    calculatedEstimator->ResetPosition(getPigeon(), getModuleStates(), pose);
}

frc::Rotation2d Drivetrain::getPigeon() {
    return pigeon.GetRotation2d();
}

void Drivetrain::resetDriveEncoders(){
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC){
    auto desiredStates = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    setSwerveDesiredState(desiredStates, true);
}

void Drivetrain::driveRobotRelative(frc::ChassisSpeeds speeds) {
    auto desiredStates = getModuleStates(speeds);
    setSwerveDesiredState(desiredStates, false);
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(units::velocity::meters_per_second_t vx_mps,
                                                                  units::velocity::meters_per_second_t vy_mps,
                                                                  units::angular_velocity::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           estimator->GetEstimatedPosition().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    return getModuleStates(chassisSpeeds);
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(frc::ChassisSpeeds chassisSpeeds)
{
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds(){
    wpi::array<frc::SwerveModuleState, 4> moduleStates = {
        swerveModules[0]->getState(),
        swerveModules[1]->getState(),
        swerveModules[2]->getState(),
        swerveModules[3]->getState()
    };
    return kinematics->ToChassisSpeeds(moduleStates);
}

void Drivetrain::setSwerveDesiredState(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates, bool isDriveOpenLoop)
{
    for (int i = 0; i < SWERVE_COUNT; i++) {
        swerveModules[i]->setDesiredState(desiredStates[i], isDriveOpenLoop);
    }
}

void Drivetrain::angleLock(){
    if (0 > getPose_m().Rotation().Degrees().to<double>()){
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) - 1.0;
    } else{
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) + 1.0;
    }
    
}

frc2::FunctionalCommand* Drivetrain::getResetOdom() {
    return new frc2::FunctionalCommand(
        [&]{ // onBegin
            for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
                aprilLime->setPipe(valor::VisionSensor::PIPELINE_0);
            }

            state.startTimestamp = frc::Timer::GetFPGATimestamp();
        },
        [&]{ // continuously running
            table->PutNumber("resetting maybe", true);

            for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
                if (aprilLime->hasTarget() && (aprilLime->getSensor().ToPose2d().X() > 0_m && aprilLime->getSensor().ToPose2d().Y() > 0_m)) {
                    table->PutNumber("resetting odom", table->GetNumber("resetting odom", 0) + 1);
                    aprilLime->applyVisionMeasurement(estimator, (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>()));
                    table->PutBoolean("resetting", true);
                    break;
                } else {
                    table->PutBoolean("resetting", false);
                }
            }
        },
        [&](bool){ // onEnd
                
        },
        [&]{ // isFinished
            return (frc::Timer::GetFPGATimestamp() - state.startTimestamp) > 1.0_s;
        },
        {}
    );
}

void Drivetrain::setXMode(){
    drive(static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::angular_velocity::radians_per_second_t>(0),true);
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.375);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.125);
    setDriveMotorNeutralMode(valor::NeutralMode::Brake);
}

void Drivetrain::setDriveMotorNeutralMode(valor::NeutralMode mode) {
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        driveControllers[i]->setNeutralMode(mode);
    }
}

frc2::InstantCommand* Drivetrain::getSetXMode(){
    frc2::InstantCommand* cmd_XMode = new frc2::InstantCommand( [&] {
        setXMode();
    });
     return cmd_XMode;
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");

        builder.AddDoubleProperty(
            "xSpeed",
            [this] { return state.xSpeed; },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeed",
            [this] { return state.ySpeed; },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeed",
            [this] { return state.rot; },
            nullptr
        );

        builder.AddDoubleProperty(
            "xSpeedMPS",
            [this] { return state.xSpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeedMPS",
            [this] { return state.ySpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeedMPS",
            [this] { return state.rotRPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "x",
            [this] { return getPose_m().X().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "y",
            [this] { return getPose_m().Y().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "theta",
            [this] { return getPose_m().Rotation().Degrees().to<double>(); },
            nullptr
        );
        builder.AddBooleanProperty(
            "swerveGood",
            [this] { return swerveNoError; },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "pose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(getPose_m().X().to<double>());
                pose.push_back(getPose_m().Y().to<double>());
                pose.push_back(getPose_m().Rotation().Degrees().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "calculatedPose",
            [this] 
            { 
                frc::Pose2d estimatedPose = calculatedEstimator->GetEstimatedPosition();
                std::vector<double> pose;
                pose.push_back(estimatedPose.X().to<double>());
                pose.push_back(estimatedPose.Y().to<double>());
                pose.push_back(estimatedPose.Rotation().Degrees().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonPitch",
            [this]
            {
                return pigeon.GetPitch().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeoYaw",
            [this]
            {
                return pigeon.GetYaw().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonRoll",
            [this]
            {
                return pigeon.GetRoll().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "stage",
            [this]
            {
                return state.stage;
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "xVelocity",
            [this]
            {
                return getRobotRelativeSpeeds().vx.to<double>();
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "yVelocity",
            [this]
            {
                return getRobotRelativeSpeeds().vy.to<double>();
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "swerve states",
            [this] 
            { 
                std::vector<double> states;
                states.push_back(swerveModules[0]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[0]->getState().speed.to<double>());
                states.push_back(swerveModules[1]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[1]->getState().speed.to<double>());
                states.push_back(swerveModules[2]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[2]->getState().speed.to<double>());
                states.push_back(swerveModules[3]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[3]->getState().speed.to<double>());
                return states;
            },
            nullptr
        );

        builder.AddDoubleProperty(
            "targetAngle",
            [this] {return (units::degree_t(state.targetAngle)).to<double>();},
            nullptr
        );

        builder.AddDoubleProperty(
            "errorAngle",
            [this] {return (units::degree_t(getAngleError()).to<double>());},
            nullptr
        );

        builder.AddDoubleProperty(
            "errorAngleRPS",
            [this] {return (state.angleRPS).to<double>();},
            nullptr
        );

        builder.AddDoubleProperty(
            "distanceFromSpeaker",
            [this] {return (state.distanceFromSpeaker).to<double>();},
            nullptr
        );
    }

