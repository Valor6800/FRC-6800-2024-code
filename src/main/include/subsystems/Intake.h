#pragma once

#include <BaseSubsystem.h>
#include "Constants.h"
#include <controllers/FalconController.h>
#include <controllers/NeoController.h>
#include <sensors/CurrentSensor.h>

#include "subsystems/Direction.h"
#include "subsystems/Position.h"
#include "subsystems/Piece.h"

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
     void init() override;

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState() override;

     void InitSendable(wpi::SendableBuilder& builder) override;

     //this state will come from elevarm, elevarm not currently connected to dev

     enum IntakeStates {
        DISABLED,
        SPIKED,
        OUTTAKE_CONE,
        OUTTAKE_CUBE,
        OUTTAKE,
        INTAKE_CONE,
        INTAKE_CUBE
     };


     struct x
     {
         IntakeStates intakeState;
         Piece pieceState;

         double intakeConeSpeed;
         double intakeCubeSpeed;
         double outtakeSpeed;
         double outtakeConeSpeed;
         double outtakeCubeSpeed;

         double cubeHoldSpeed;
         double coneHoldSpeed;

         double coneSpikeCurrent;
         double cubeSpikeCurrent;

         double coneCacheSize;
         double cubeCacheSize;

         bool intakeOp;

     }state, prevState;
    
    std::unordered_map<std::string, IntakeStates> stringToStateMap = {
        {"disable", IntakeStates::DISABLED},
        {"spiked", IntakeStates::SPIKED},
        {"outtake_cone", IntakeStates::OUTTAKE_CONE},
        {"outtake_cube", IntakeStates::OUTTAKE_CUBE},
        {"outtake", IntakeStates::OUTTAKE},
        {"intake_cone", IntakeStates::INTAKE_CONE},
        {"intake_cube", IntakeStates::INTAKE_CUBE}
     };
     
     IntakeStates stringToIntakeState(std::string name){
        if (!stringToStateMap.contains(name))
               return IntakeStates::DISABLED;
        return stringToStateMap.at(name);
     }

     frc2::FunctionalCommand * getAutoCommand(std::string);

private:

     valor::FalconController intakeMotor;

     valor::CurrentSensor currentSensor;
};