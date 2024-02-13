/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <iostream>
#include <frc/RobotController.h>
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include "units/length.h"
#include <string>
#include <vector>
#include <cscore.h>
#include <frc/geometry/Pose3d.h>
#include "networktables/NetworkTable.h"


#define ALPHA_TEAM_NUMBER 6801
#define SIDE_SWIPE_TEAM_NUMBER 6808
// When trying to compile against other targets for simulation, cmath doesn't include M_PI
//   Therefore if not defined, define M_PI for use on other targets
#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

namespace DIOPorts {
    constexpr static int MAG_ENCODER_PORTS[4] = {1, 2, 3, 4};
    constexpr static int BEAM_BREAK_PORT = 0;
}

namespace CANIDs {
    constexpr static int DRIVE_CANS[4] = {2, 4, 6, 8};
    constexpr static int AZIMUTH_CANS[4] = {1, 3, 5, 7};
    constexpr static int CANCODER_CANS[4] = {20, 21, 22, 23};
    constexpr static int PIGEON_CAN = 61;
    constexpr static int CANDLE = 60;
    constexpr static int INTERNAL_INTAKE = 13;
    constexpr static int EXTERNAL_INTAKE = 9;
    constexpr static int EXTERNAL_DEPLOYMENT = 19; // rando number
    constexpr static int PIVOT = 20; // rando number
    constexpr static int RIGHT_SHOOTER_WHEEL_CONTROLLER = 10;
    constexpr static int LEFT_SHOOTER_WHEEL_CONTROLLER = 11;
    constexpr static int FEEDER = 12;
    constexpr static int RIGHT_CLIMBER = 24; // rando number
    constexpr static int LEFT_CLIMBER = 25; // rando number
}

// Constants that stay the same across bots should not go here
namespace Constants { 
    // public:
        /*
        To use a global Constants:: instead of using an object, the function/variable used must be "static".
        The reason that can't happen here is that for functions/variables to be static, the values they use must also be static.
        Because GetTeamNumber isn't static, team number can't be static, and therefore none of the getters can be static either. 
        */
        static int teamNumber = frc::RobotController::GetTeamNumber();;
        // void getMacAddress(){
        //     std::vector<std::string> v = cs::GetNetworkInterfaces();
        //     nt::NetworkTableInstance::GetDefault().GetTable("constants maybe")
        // }

        static units::degree_t pigeonMountPitch(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return 0_deg; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg;  
            default: return 0_deg; // Temp value; TODO: Change it
        }};
        static units::degree_t pigeonMountRoll(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -0.395508_deg;
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg; // Temp value; TODO: Change it  
            default: return 0_deg; // Temp value; TODO: Change it 
        }};
        static units::degree_t pigeonMountYaw(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -1.477661_deg; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg; // Temp value; TODO: Change it  
            default: return 0_deg; // Temp value; TODO: Change it 
        }};

        static std::vector<double> swerveZeros(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {0.3867, 0.8890, 0.0763, 0.610};
            case SIDE_SWIPE_TEAM_NUMBER: return {0.3106, 0.4369, 0.4780, 0.7372};
            default: return {0.4240722, 0.857666, 0.46378, 0.078125};
        }};

        static units::meter_t moduleDiff(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.2413_m;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.2_m;
            default: return 0.295_m;
        }};
        static units::meter_t driveBaseRadius(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.36_m; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0.3_m; // Temp value; TODO: Change it
            default: return 0.3_m; // Temp number; TODO: Change it
        }};

        static std::vector<bool> swerveDrivesReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, false, false};
            case SIDE_SWIPE_TEAM_NUMBER: return {false, false, false, false};
            default: return {true, false, false, false};
        }};
        static std::vector<bool> swerveAzimuthsReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, true, true};
            case SIDE_SWIPE_TEAM_NUMBER: return {true, true, true, true};
            default: return {true, false, true, true};
        }};

        static double azimuthKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 0.00001;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.00001;
            default: return 10.0;
        }};
        static double azimuthKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 10.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 10.0;
            default: return 6.0;
        }};
        static double azimuthKAcc(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 200.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 200.0;
            default: return 130.0;
        }};

        static double driveKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 0.001;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.001;
            default: return 0.00001;
        }};
        static double driveKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 6.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 6.0;
            default: return 5.0;
        }};
        static double driveKAcc(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 120.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 120.0;
            default: return 50.0;
        }};

        static frc::Pose3d mintCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d{
                0.2921_m, //x
                -0.16906875_m, //y
                0.28575_m, //z
                frc::Rotation3d{
                    -180_deg, 
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            default: return frc::Pose3d{
                0.2921_m, //x
                -0.16906875_m, //y
                0.28575_m, //z
                frc::Rotation3d{
                    -180_deg, 
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
        }};

        static frc::Pose3d vanillaCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d(); // Temp value; TODO: Change it
            default: return frc::Pose3d();
        }};

        static frc::Pose3d chocolateCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d lemonCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            };
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            default: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
        }};

        static std::vector<std::pair<const char*, frc::Pose3d>> aprilCameras{
                    std::pair("limelight-mint", mintCameraPosition()),
                    std::pair("limelight-lemon", lemonCameraPosition()),
                    std::pair("limelight-choco", chocolateCameraPosition()),
                    std::pair("limelight-vanilla", vanillaCameraPosition())
        };
}
