#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/FalconController.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/sensors/CurrentSensor.h"

#include "Direction.h"
#include "Position.h"
#include "Piece.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>

/**
 * @brief Subsystem - Intake
 */
class Intake : public valor::BaseSubsystem
{
public:
     /**
      * @brief Construct a new Intake object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Intake(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Intake object
      * 
      * Intake objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Intake();

     /**
      * @brief Initialize the Intake
      * 
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     void InitSendable(wpi::SendableBuilder& builder);

     


     //this state will come from elevarm, elevarm not currently connected to dev

     enum IntakeStates {
        DISABLED, // 0
        SPIKED, // 1
        OUTTAKE, // 2
        INTAKE // 3
     };


     struct x
     {
         IntakeStates intakeState;
         Piece pieceState;

         double intakeConeSpeed;
         double intakeCubeSpeed;
         double outtakeConeSpeed;
         double outtakeCubeSpeed;

         double cubeHoldSpeed;
         double coneHoldSpeed;

         double coneSpikeCurrent;
         double cubeSpikeCurrent;

         double coneCacheSize;
         double cubeCacheSize;

         bool intakeOp;

         bool elevarmGround;
         bool elevarmPoopFull;

         bool isCubeStall;
         
     }state, prevState;
    
    std::unordered_map<std::string, IntakeStates> stringToStateMap = {
        {"disabled", IntakeStates::DISABLED},
        {"spiked", IntakeStates::SPIKED},
        {"outtake", IntakeStates::OUTTAKE},
        {"intake", IntakeStates::INTAKE}
     };

     Piece getFuturePiece();
     Piece getPrevPiece();
     
     IntakeStates stringToIntakeState(std::string name){
        if (!stringToStateMap.contains(name))
               return IntakeStates::DISABLED;
        return stringToStateMap.at(name);
     }
     
      std::unordered_map<std::string, Piece> stringToPieceMap = {
            {"cone", Piece::CONE},
            {"cube", Piece::CUBE}
         };

      Piece stringToPieceState(std::string piece) {
            if (!stringToPieceMap.contains(piece))
                     return Piece::CONE;
            return stringToPieceMap.at(piece);
      }

     frc2::FunctionalCommand * getAutoCommand(std::string, std::string);

   void setConeHoldSpeed(double isAuto);

private:

     valor::FalconController intakeMotor;

     valor::CurrentSensor currentSensor;
};