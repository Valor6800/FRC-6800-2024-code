#include "valkyrie/Auto.h"
// #include <frc/Filesystem.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <filesystem>

using namespace valor;
using namespace pathplanner;

#define AUTOS_PATH (std::string)"/home/lvuser/deploy/pathplanner/autos/"
#define PATHS_PATH (std::string)"/home/lvuser/deploy/pathplanner/paths/"

#define MAX_VEL (units::meters_per_second_t) 3.0
#define MAX_ACCEL (units::meters_per_second_squared_t ) 4.0
#define MAX_ANG_VEL (units::radians_per_second_t) 540
#define MAX_ANG_ACCEL (units::radians_per_second_squared_t) 720.0

Auto::Auto(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("auto");
}

frc2::CommandPtr Auto::makeAuto(std::string autoName){
    // return pathplanner::PathPlannerAuto(autoName).ToPtr();
    std::vector<std::shared_ptr<PathPlannerPath>> paths = PathPlannerAuto::getPathGroupFromAutoFile(autoName);
    pathCommands.clear();
    for (int i = 0; i < paths.size(); i++) {
        frc2::CommandPtr moveCmd = AutoBuilder::followPathWithEvents(paths[paths.size() - 1 - i]);
        if (i != 0){
            pathCommands.push_back(std::move(moveCmd).AndThen(std::move(pathCommands[i - 1])));
        } else {
            pathCommands.push_back(std::move(moveCmd));
        }

    }
    // as a result, pathCommands[1] will be missing the first path, pathComannds[2] the first and second, etc. 
    std::reverse(pathCommands.begin(), pathCommands.end());
    return std::move(pathCommands[0]);
}

frc2::CommandPtr Auto::getCurrentAuto(){
    std::string selection = m_chooser.GetSelected(); 
    return makeAuto(selection);
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
    if (false == true) {
        //how???;
    }
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
