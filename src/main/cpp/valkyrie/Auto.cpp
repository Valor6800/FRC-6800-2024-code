#include "valkyrie/Auto.h"
// #include <frc/Filesystem.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
// 
#include <filesystem>

using namespace valor;
using namespace pathplanner;

#define AUTOS_PATH (std::string)"/home/lvuser/deploy/pathplanner/autos/"
#define PATHS_PATH (std::string)"/home/lvuser/deploy/pathplanner/paths/"

#define AUTO_MAX_ACCEL units::meters_per_second_squared_t(0.0)
#define AUTO_MAX_VEL units::meters_per_second_t(0.0)
#define AUTO_MAX_ANG_ACCEL units::radians_per_second_squared_t(0.0)
#define AUTO_MAX_ANG_VEL units::radians_per_second_t(0.00f)

Auto::Auto(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("auto");
}

frc2::CommandPtr Auto::makeAuto(std::string autoName){
    return pathplanner::PathPlannerAuto(autoName).ToPtr();
}

frc2::CommandPtr Auto::getCurrentAuto(){
    std::string selection = m_chooser.GetSelected(); 
    return makeAuto(selection);
}

std::string Auto::getAutoName(){
    return m_chooser.GetSelected();
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

std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (const auto & entry : std::filesystem::directory_iterator(path_name)){
        std::string path = entry.path();
        if (path.find(".path") != std::string::npos || path.find(".auto") != std::string::npos) {
            files.push_back(entry.path());
        }
    }
    return files;
}

void Auto::fillAutoList(){
    // for (std::string path : listDirectory(PATHS_PATH)){
    //     m_chooser.AddOption(makeFriendlyName(removeFileType(path)), removeFileType(path));
    for (std::string path : listDirectory(AUTOS_PATH)){
        m_chooser.AddOption(makeFriendlyName(removeFileType(path)), removeFileType(path));
    }
    frc::SmartDashboard::PutData(&m_chooser);
}

PathConstraints constraints = PathConstraints(
    AUTO_MAX_VEL,
    AUTO_MAX_ACCEL,
    AUTO_MAX_ANG_VEL,
    AUTO_MAX_ANG_ACCEL
);

frc2::CommandPtr Auto::getPathFindToPose(frc::Pose2d targetPose, units::meters_per_second_t endVelocity, units::meter_t rotDelay){
    frc2::CommandPtr pathfindingCommand = AutoBuilder::pathfindToPose(
        targetPose,
        constraints,
        endVelocity,
        rotDelay
    );
    return pathfindingCommand;
}

frc2::CommandPtr Auto::getFollowPathFind(std::shared_ptr<PathPlannerPath> path, units::meter_t rotDelay){
    frc2::CommandPtr pathfindingCommand = AutoBuilder::pathfindThenFollowPath(
        path, 
        constraints,
        rotDelay
    );
    return pathfindingCommand;
}

std::vector<frc::Pose2d> Auto::generatePoses(frc::Pose2d startPose, frc::Pose2d endPose, bool useLimelight){
    if(!useLimelight){
        return std::vector<frc::Pose2d> {startPose, endPose};
    }
}

std::shared_ptr<PathPlannerPath> Auto::makePath(std::vector<frc::Pose2d> poses, units::meters_per_second_t endMPS, units::degree_t endRot){
    std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(poses);

    auto path = std::make_shared<PathPlannerPath>(
        bezierPoints,
        constraints, 
        GoalEndState(endMPS, endRot)
    );

    return path;
}

frc2::CommandPtr Auto::makeCommandFromPath(std::shared_ptr<PathPlannerPath> path){
    return AutoBuilder::followPath(path);
}

std::vector<frc2::CommandPtr> Auto::makeOTFAuto(std::string autoName){
    if(autoName.find("OTF") != autoName.npos){
        std::vector<std::shared_ptr<PathPlannerPath>> paths = PathPlannerAuto::getPathGroupFromAutoFile(autoName);
        std::vector<std::shared_ptr<PathPlannerPath>> newPaths = paths;
        for(int i = 0; i < paths.size() - 1; i++){
            if(comparePose2D(getEndPoseFromPath(paths[i]), paths[i+1].get()->getStartingDifferentialPose())){
                // put path gen code
                frc::Pose2d targetedOTFEndPose = getOTFEndPose(false, units::meter_t(5), units::meter_t(7), units::degree_t(0));
                std::vector<frc::Pose2d> poses = {getEndPoseFromPath(paths[i]), targetedOTFEndPose, getEndPoseFromPath(paths[i+1])};
                std::shared_ptr<PathPlannerPath> newTraj = makePath(poses, 0_mps, 0_deg);
                // stitch together the paths to form a new autonomous
                newPaths.erase(newPaths.begin() + i);
                newPaths.insert(newPaths.begin() + i, newTraj);
            }
        }
        std::vector<frc2::CommandPtr> commands;
        for(int i = 0; i < newPaths.size(); i++){
            commands.insert(commands.begin() + i, AutoBuilder::followPathWithEvents(newPaths[i]));
        }
        return commands;
    }
}

bool Auto::comparePose2D(frc::Pose2d onePose, frc::Pose2d twoPose){
    if(onePose.X() == twoPose.X() && onePose.Y() == twoPose.Y()){
        return true;
    }
    return false;
}

frc::Pose2d Auto::getEndPoseFromPath(std::shared_ptr<PathPlannerPath> path){
    auto X = path.get()->getAllPathPoints().back().position.X();
    auto Y = path.get()->getAllPathPoints().back().position.Y();
    auto theta = path.get()->getGoalEndState().getRotation();
    frc::Pose2d endPose = frc::Pose2d{X, Y, theta};
    return endPose;
}

frc::Pose2d Auto::getOTFEndPose(bool limelight, units::meter_t X, units::meter_t Y, units::degree_t rot){
    frc::Pose2d pose = {X, Y, rot};
    if(limelight){
        return pose; // will be the pose of the object
    }
    return pose;
}

void Auto::scheduleCommands(std::vector<frc2::CommandPtr> commands){
    for (int i = 0; i < commands.size(); i++){
        commands.at(i).get()->Schedule();
    }
}