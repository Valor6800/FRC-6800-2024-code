#include "auto/ValorAuto.h"
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <wpi/ghc/filesystem.hpp>

#define PATHS_PATH (std::string)"/home/lvuser/deploy/pathplanner/"

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Intake *_intake, Elevarm *_elevarm) :
    drivetrain(_drivetrain), intake(_intake), elevarm(_elevarm)
{
    table = nt::NetworkTableInstance::GetDefault().GetTable("auto");
    drivetrain->getTrajectoryConfig().SetKinematics(*drivetrain->getKinematics());
    eventMap = {
        {"stow_low", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
                elevarm->futureState.positionState = Position::STOW;
        }))},
        {"outtake_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            elevarm->setFuturePiece(Piece::CUBE);
            intake->state.intakeState = Intake::IntakeStates::OUTTAKE;
        }))},
        {"intake_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            elevarm->setFuturePiece(Piece::CUBE);
            intake->state.intakeState = Intake::IntakeStates::INTAKE;
        }))},
        {"outtake_cone", std::shared_ptr<frc2::Command>(new frc2::SequentialCommandGroup(
            frc2::InstantCommand([&]{
                table->PutString("running command", "outtake_cone");
                elevarm->setFuturePiece(Piece::CONE);
                intake->state.intakeState = Intake::IntakeStates::OUTTAKE;
            }),
            frc2::WaitCommand(850_ms),
            frc2::InstantCommand([&]{
                intake->state.intakeState = Intake::IntakeStates::DISABLED;
            })
        ))},
        {"score_final_cube", std::shared_ptr<frc2::Command>(new frc2::SequentialCommandGroup(
            frc2::InstantCommand([&]{
                elevarm->setFuturePiece(Piece::CUBE);
                intake->state.intakeState = Intake::IntakeStates::OUTTAKE;
            }),
            frc2::WaitCommand(150_ms),
            frc2::InstantCommand([&]{
                intake->state.intakeState = Intake::IntakeStates::DISABLED;
                elevarm->futureState.positionState = Position::SNAKE;
                elevarm->futureState.directionState = Direction::BACK;
            })
        ))},
        {"disable_intake", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            intake->state.intakeState = Intake::IntakeStates::DISABLED;
        }))},
        {"score_mid_cone", std::shared_ptr<frc2::Command>(new frc2::SequentialCommandGroup(
            std::move(*elevarm->getAutoCommand("cone", "front", "mid")),
            frc2::WaitCommand(150_ms),
            frc2::InstantCommand([&]{
                intake->state.intakeState = Intake::IntakeStates::DISABLED;
            })
        ))},
        {"pickup_back_cone", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            table->PutString("running command", "pickup_back_cone");
            elevarm->setFuturePiece(Piece::CONE);
            elevarm->futureState.positionState = Position::GROUND_TOPPLE;
            elevarm->futureState.directionState = Direction::BACK;
            intake->state.intakeState = Intake::IntakeStates::INTAKE;
        }))},
        {"pickup_back_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            table->PutString("running command", "pickup_back_cube");
            elevarm->setFuturePiece(Piece::CUBE);
            elevarm->futureState.positionState = Position::GROUND;
            elevarm->futureState.directionState = Direction::BACK;
            intake->state.intakeState = Intake::IntakeStates::INTAKE;
        }))},
        {"pickup_front_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            table->PutString("running command", "pickup_front_cube");
            elevarm->setFuturePiece(Piece::CUBE);
            elevarm->futureState.positionState = Position::GROUND;
            elevarm->futureState.directionState = Direction::FRONT;
            intake->state.intakeState = Intake::IntakeStates::INTAKE;
        }))},
        {"pickup_front_cube_interrupting", std::shared_ptr<frc2::Command>( 
            new frc2::SequentialCommandGroup(
                frc2::InstantCommand([&] {
                    elevarm->setFuturePiece(Piece::CUBE);
                    intake->state.intakeState = Intake::IntakeStates::INTAKE;
                }),
                std::move(*elevarm->getAutoCommand("cube", "front", "ground"))
        ))},
        {"stage_mid_cone", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            elevarm->setFuturePiece(Piece::CONE);
            elevarm->futureState.positionState = Position::MID;
            elevarm->futureState.directionState = Direction::FRONT;
        }))},
        {"stage_mid_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            elevarm->setFuturePiece(Piece::CUBE);
            elevarm->futureState.positionState = Position::MID;
            elevarm->futureState.directionState = Direction::FRONT;
        }))},
        {"stow_high_cone", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            table->PutString("running command", "stow_high_cone");
            elevarm->setFuturePiece(Piece::CONE);
            elevarm->futureState.positionState = Position::SNAKE;
            elevarm->futureState.directionState = Direction::BACK;
        }))},
        {"stow_high_cube", std::shared_ptr<frc2::Command>(new frc2::InstantCommand([&] {
            table->PutString("running command", "stow_high_cube");
            elevarm->setFuturePiece(Piece::CUBE);
            elevarm->futureState.positionState = Position::SNAKE;
            elevarm->futureState.directionState = Direction::BACK;
        }))}
    };


}

ValorAuto::~ValorAuto()
{
}

frc2::Command * ValorAuto::createPPTrajectoryCommand(pathplanner::PathPlannerTrajectory trajectory){

    frc2::PIDController thetaController = frc2::PIDController(drivetrain->getThetaPIDF().P, drivetrain->getThetaPIDF().I, drivetrain->getThetaPIDF().D);
    thetaController.EnableContinuousInput(units::radian_t(-M_PI).to<double>(),
                                          units::radian_t(M_PI).to<double>());

    return new pathplanner::PPSwerveControllerCommand(
        trajectory,
        [&] () { return drivetrain->getPose_m(); },
        *drivetrain->getKinematics(),
        frc2::PIDController(drivetrain->getXPIDF().P, drivetrain->getXPIDF().I, drivetrain->getXPIDF().D),
        frc2::PIDController(drivetrain->getYPIDF().P, drivetrain->getYPIDF().I, drivetrain->getYPIDF().D),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain},
        true
    );
}

frc2::CommandPtr ValorAuto::makeAuto(std::string autoName) {
    // Load path group automatically splits the trajectory by stop points, and so we call our search 
    // and grab function between these trajectories

    std::vector<pathplanner::PathPlannerTrajectory> trajectories = pathplanner::PathPlanner::loadPathGroup(
        autoName,
        units::meters_per_second_t(drivetrain->getAutoMaxSpeed()),
        units::meters_per_second_squared_t(drivetrain->getAutoMaxAcceleration())
    );
    
    trajectoryCommands.clear();

    // The idea is to build out the complete auto by building a sort of recursive command.
    // Upon completion of the 1st trajectory command, the attached function first schedules
    // the drive command, which has a function to schedule the 2nd trajectory command. Upon completion 
    // of the 2nd trajectory command, the attached function schedules another drive command which 
    // has a function to schedule the 3rd trajectory command and so on.

    std::vector<pathplanner::PathPlannerTrajectory> realTrajectories; 

    for (uint i = 0; i < trajectories.size(); i++){
        if (i % 2 == 1) // Every second odd-indexed trajectory should be the inbetweener
            continue;
        realTrajectories.push_back(trajectories[i]);
        frc2::Command * moveCommand = new pathplanner::FollowPathWithEvents (
            std::unique_ptr<frc2::Command>(createPPTrajectoryCommand(trajectories[i])),
            trajectories[i].getMarkers(),
            eventMap
        );
        trajectoryCommands.push_back(std::move(*(moveCommand)).ToPtr());
    }

    for (int i = trajectoryCommands.size() - 2; i >= 0; i--){
        pathplanner::PathPlannerTrajectory::PathPlannerState 
        initState = realTrajectories[i].getEndState(),
        nextState = realTrajectories[i + 1].getInitialState();
        trajectoryCommands[i] = std::move(trajectoryCommands[i]).AndThen(
            [this, i, initState, nextState](){
                frc::Translation2d targetPos = drivetrain->state.currentGamePiece.globalPosition;
                driveCommand = std::move(*(drivetrain->getOTFDriveCommand(targetPos, initState, nextState))).ToPtr()
                .AndThen([this, i](){
                    trajectoryCommands[i + 1].Schedule();
                });
                driveCommand.Schedule();
            }, 
            {drivetrain}
        );
    }
    std::vector<frc2::CommandPtr> cmdSequence;
    cmdSequence.push_back(frc2::InstantCommand([this, trajectories] () {
            drivetrain->resetOdometry(trajectories[0].getInitialHolonomicPose());
        }, {drivetrain}).ToPtr()
    );
    for (std::string name : trajectories[0].getStartStopEvent().names) {
        cmdSequence.push_back(std::move(*eventMap[name]).ToPtr());
    }
    cmdSequence.push_back(std::move(trajectoryCommands[0]));
    for (std::string name : trajectories.back().getEndStopEvent().names) {
        cmdSequence.push_back(std::move(*eventMap[name]).ToPtr());
    } 

    std::vector<std::string> nonExistentCommands = {};
    for (pathplanner::PathPlannerTrajectory trajectory : trajectories) {
        for (pathplanner::PathPlannerTrajectory::EventMarker event : trajectory.getMarkers()){
            for (std::string name : event.names) {
                if (eventMap[name] == nullptr) {
                    nonExistentCommands.push_back(name);
                }
            }
        }
    }
    table->PutStringArray("non existent commnands", std::span<std::string>{nonExistentCommands});
    return frc2::cmd::Sequence(std::move(cmdSequence));
}

frc2::CommandPtr ValorAuto::getCurrentAuto(){
    std::string selection = m_chooser.GetSelected(); 
    return makeAuto(selection);
}

std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (auto &entry : ghc::filesystem::directory_iterator(path_name)) {
        std::string path = entry.path();
        if (path.find(".path") != std::string::npos || path.find(".csv") != std::string::npos) {
            files.push_back(entry.path());
        }
    }
    return files;
}

std::string removeFileType(std::string fileName) {
    return fileName.substr(fileName.find_last_of('/') + 1, fileName.find_last_of('.') - fileName.find_last_of('/') - 1);
}

bool is_alpha(char c){
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
bool is_caps(char c){
    return (c >= 'A' && c <= 'Z');
}

std::string makeFriendlyName(std::string filename){
    // take last part of the path string when divided with /'s - this should be the filename
    filename = filename.substr(filename.find_last_of('/') + 1);
    std::string n_name = "";
    for (uint i = 0; i < filename.length(); i ++){
        // .'s signify the end of the filename and the start of the file extension
        if (filename[i] == '.'){
            break;
        } else if (filename[i] == '_'){ // replace _'s with spaces for a snake case filename
            // make sure we dont have double spaces
            if (*(n_name.end() - 1) != ' ')
                n_name += ' ';
        } else if (i >= 1 && is_alpha(filename[i]) && is_caps(filename[i]) && !is_caps(filename[i - 1]) && *(n_name.end() - 1) != ' '){ // check for camel case, add space if present
            n_name += ' ';
            n_name += tolower(filename[i]);
        } else if (i == 0){ // first letter should be capitaized
            n_name += toupper(filename[i]);
        } else{
            n_name += tolower(filename[i]);
        }
    }
    return n_name;
}

void ValorAuto::fillAutoList(){
    for (std::string path : listDirectory(PATHS_PATH)){
        m_chooser.AddOption(makeFriendlyName(removeFileType(path)), removeFileType(path));
    }
    frc::SmartDashboard::PutData(&m_chooser);
}
