#include "Drivetrain.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include "valkyrie/sensors/VisionSensor.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"

using namespace pathplanner;

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
// #define KP_LOCK 0.2f
#define KP_LIMELIGHT 0.7f

#define LIMELIGHT_X 0.22225f //meters
#define LIMELIGHT_Y 0.3302f //meters
#define LIMELIGHT_Z 0.62865f //meters

#define LIMELIGHT_ROLL 0.0f //degrees
#define LIMELIGHT_PITCH 0.0f //degrees
#define LIMELIGHT_YAW 0.0f //degrees

#define KPX 60.0f //50
#define KIX 0.0f //0
#define KDX 0.0f //.1
#define KFX 0.0f

#define KPY 60.0f //65
#define KIY 0.0f //0
#define KDY 0.0f //.1
#define KFY 0.0f

#define KPT 15.0f
#define KIT 0.0f
#define KDT 0.0f
#define KFT 0.0f

#define AZIMUTH_K_P 0.00001f
#define AZIMUTH_K_I 0.0f
#define AZIMUTH_K_D 0.0f
#define AZIMUTH_K_E 0.0027f

#define AZIMUTH_K_VEL 10.0f
#define AZIMUTH_K_ACC_MUL 0.05f

#define DRIVE_K_P 0.001f
#define DRIVE_K_I 0.0f
#define DRIVE_K_D 0.0f
#define DRIVE_K_E 0.0027f

#define DRIVE_K_VEL 6.0f
#define DRIVE_K_ACC_MUL 0.05f

#define MOTOR_FREE_SPEED 6380.0f
#define WHEEL_DIAMETER_M 0.0973f //0.1016
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define AUTO_MAX_SPEED 10.0f
#define AUTO_MAX_ACCEL_SECONDS 5.33f //5.33
#define ROT_SPEED_MUL 2.0f

#define AUTO_VISION_THRESHOLD 4.0f //meters

#define X_TIME 214.85f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS ""
#define PIGEON_CAN_BUS "baseCAN"

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Drivetrain"),
                        driveMaxSpeed(MOTOR_FREE_SPEED / 60.0 / DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        autoMaxSpeed(AUTO_MAX_SPEED),
                        autoMaxAccel(AUTO_MAX_SPEED/AUTO_MAX_ACCEL_SECONDS),
                        rotMaxAccel(rotMaxSpeed * 0.5),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        initPositions(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        config(NULL),
                        swerveNoError(true),
                        aprilLL(_robot, "limelight-vanilla", frc::Pose3d{
                            (units::length::meter_t) LIMELIGHT_X,
                            (units::length::meter_t) LIMELIGHT_Y,
                            (units::length::meter_t) LIMELIGHT_Z,
                            frc::Rotation3d{
                                (units::degree_t) LIMELIGHT_ROLL,
                                (units::degree_t) LIMELIGHT_PITCH,
                                (units::degree_t) LIMELIGHT_YAW,
                            }
                        }),
                        aprilChocolate(_robot, "limelight-choco", frc::Pose3d{
                            (units::length::meter_t) -LIMELIGHT_X,
                            (units::length::meter_t) LIMELIGHT_Y,
                            (units::length::meter_t) LIMELIGHT_Z,
                            frc::Rotation3d{
                                (units::degree_t) LIMELIGHT_ROLL,
                                (units::degree_t) LIMELIGHT_PITCH,
                                (units::degree_t) LIMELIGHT_YAW,
                            }
                        })
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
    delete config;
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{constants.moduleDiff() * MDX[i],
                                           constants.moduleDiff() * MDY[i]};

    valor::PIDF azimuthPID;
    azimuthPID.velocity = AZIMUTH_K_VEL;
    azimuthPID.acceleration = AZIMUTH_K_ACC_MUL;
    azimuthPID.P = AZIMUTH_K_P;
    azimuthPID.I = AZIMUTH_K_I;
    azimuthPID.D = AZIMUTH_K_D;
    azimuthPID.error = AZIMUTH_K_E;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      valor::NeutralMode::Brake,
                                                      true,
                                                      DRIVETRAIN_CAN_BUS));
    azimuthControllers[i]->setConversion(1.0 / AZIMUTH_GEAR_RATIO);
    azimuthControllers[i]->setPIDF(azimuthPID, 0);

    valor::PIDF drivePID;
    drivePID.velocity = DRIVE_K_VEL;
    drivePID.acceleration = DRIVE_K_ACC_MUL;
    drivePID.P = DRIVE_K_P;
    drivePID.I = DRIVE_K_I;
    drivePID.D = DRIVE_K_D;
    drivePID.error = DRIVE_K_E;

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    valor::NeutralMode::Coast,
                                                    false,
                                                    DRIVETRAIN_CAN_BUS));
    driveControllers[i]->setConversion(1.0 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M);
    driveControllers[i]->setPIDF(drivePID, 0);

    swerveModules.push_back(new valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);
    

}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    pullSwerveModuleZeroReference();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    aprilLL.setPipe(valor::VisionSensor::PIPELINE_0);

    initPositions.fill(frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)});

    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        configSwerveModule(i);
    }

    pigeon.GetConfigurator().Apply(
        ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose(
            ctre::phoenix6::configs::MountPoseConfigs{}
            .WithMountPosePitch(constants.pigeonMountPitch().to<double>())
            .WithMountPoseRoll(constants.pigeonMountRoll().to<double>())
            .WithMountPoseYaw(constants.pigeonMountYaw().to<double>())
        )
    );

    kinematics = new frc::SwerveDriveKinematics<SWERVE_COUNT>(motorLocations);
    estimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), initPositions, frc::Pose2d{0_m, 0_m, 0_rad});
    config = new frc::TrajectoryConfig(units::velocity::meters_per_second_t{autoMaxSpeed}, units::acceleration::meters_per_second_squared_t{autoMaxAccel});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;
    xPIDF.F = KFX;

    yPIDF.P = KPY;
    yPIDF.I = KIY;
    yPIDF.D = KDY;
    yPIDF.F = KFY;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;
    thetaPIDF.F = KFT;

    table->PutNumber("Vision Std", 3.0);
    table->PutBoolean("Load Swerve Mag Encoder", false);

    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);

    state.lock = false;

    resetState();

    AutoBuilder::configureHolonomic(
        [this](){ return getPose_m(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(getXPIDF().P, getXPIDF().I, getXPIDF().D), // Translation PID constants
            PIDConstants(getThetaPIDF().P, getThetaPIDF().I, getThetaPIDF().D), // Rotation PID constants
            units::meters_per_second_t{driveMaxSpeed}, // Max module speed, in m/s
            constants.driveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
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
    if (!driverGamepad) return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }

    state.adas = driverGamepad->GetAButton();
    state.topTape = operatorGamepad->DPadUp();
    state.bottomTape = operatorGamepad->DPadRight();
    state.lock = state.adas || driverGamepad->GetBButton();

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);
    if (!state.lock){
    state.rot = driverGamepad->rightStickX(3);
    }

    state.xPose = driverGamepad->GetXButton();
}

void Drivetrain::analyzeDashboard()
{
    if (table->GetBoolean("Load Swerve Mag Encoder",false))
        pullSwerveModuleZeroReference();

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                            getPigeon(),
                            {
                                swerveModules[0]->getModulePosition(),
                                swerveModules[1]->getModulePosition(),
                                swerveModules[2]->getModulePosition(),
                                swerveModules[3]->getModulePosition()
                            });

    if (aprilLL.hasTarget()) {
        frc::Pose2d botpose = aprilLL.getSensor().ToPose2d();
            // estimator->AddVisionMeasurement(
            //     state.visionPose,  
            //     frc::Timer::GetFPGATimestamp(),
            //     {visionStd, visionStd, visionStd}
            // ); 
        
        
        if (driverGamepad->GetStartButton()){
            resetOdometry(botpose);
        }
    }
}

void Drivetrain::assignOutputs()
{    
    if (state.lock){angleLock();}
    state.xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    state.ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    state.rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};

    if (state.xPose){
        setXMode();
    } else if (state.adas){
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    } 
    else {
        setDriveMotorNeutralMode(valor::NeutralMode::Coast);
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    }
}

void Drivetrain::pullSwerveModuleZeroReference(){
    swerveNoError = true;
    for (size_t i = 0; i < swerveModules.size(); i++) {
        swerveNoError &= swerveModules[i]->loadAndSetAzimuthZeroReference(constants.swerveZeros());
    }
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
}

void Drivetrain::addVisionMeasurement(frc::Pose2d visionPose, double doubt=1){
    if (aprilLL.hasTarget())   
        estimator->AddVisionMeasurement(
            visionPose,  
            frc::Timer::GetFPGATimestamp(),
            {doubt, 999999.0, 999999.0}
        ); 
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{

    aprilLL.setPipe(valor::VisionSensor::PIPELINE_0);

    wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, SWERVE_COUNT>(wpi::empty_array);

    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }

    estimator->ResetPosition(getPigeon(), modulePositions, pose);
}

frc::Rotation2d Drivetrain::getPigeon() 
{
    return pigeon.GetRotation2d();
}

void Drivetrain::resetDriveEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC)
{
    auto states = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

void Drivetrain::driveRobotRelative(frc::ChassisSpeeds speeds) {
    auto states = getModuleStates(speeds);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
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
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
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

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates)
{ 
    kinematics->DesaturateWheelSpeeds(&desiredStates, units::velocity::meters_per_second_t{autoMaxSpeed});
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], false);
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
            aprilLL.setPipe(valor::VisionSensor::PIPELINE_0);
            state.startTimestamp = frc::Timer::GetFPGATimestamp();
        },
        [&]{ // continuously running
            table->PutNumber("resetting maybe", true);
            if (aprilLL.hasTarget() && (aprilLL.getSensor().ToPose2d().X() > 0_m && aprilLL.getSensor().ToPose2d().Y() > 0_m)){
                table->PutNumber("resetting odom", table->GetNumber("resetting odom", 0) + 1);
                addVisionMeasurement(aprilLL.getSensor().ToPose2d(), 1.0);
                table->PutBoolean("resetting", true);
            }
            else {
                table->PutBoolean("resetting", false);
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

double Drivetrain::getDriveMaxSpeed() {
    return driveMaxSpeed;
}

double Drivetrain::getAutoMaxSpeed() {
    return autoMaxSpeed;
}

double Drivetrain::getAutoMaxAcceleration() {
    return autoMaxAccel;
}


double Drivetrain::getRotationMaxSpeed() {
    return rotMaxSpeed;
}

double Drivetrain::getRotationMaxAcceleration() {
    return rotMaxAccel;
}

frc::TrajectoryConfig & Drivetrain::getTrajectoryConfig() {    
    return *config;
}

valor::PIDF Drivetrain::getXPIDF() {
    return xPIDF;
}

valor::PIDF  Drivetrain::getYPIDF() {
    return yPIDF;
}

valor::PIDF Drivetrain::getThetaPIDF() {
    return thetaPIDF;
}

void Drivetrain::setAutoMaxAcceleration(double acceleration, double multiplier)  {

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

        builder.AddBooleanProperty("Target?", [this] {return aprilLL.hasTarget();}, nullptr);

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
            "prevPose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(state.prevPose.X().to<double>());
                pose.push_back(state.prevPose.Y().to<double>());
                pose.push_back(state.prevPose.Rotation().Degrees().to<double>());
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
    }

