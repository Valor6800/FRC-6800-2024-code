# pragma once 

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"

class Indexer : public valor::BaseSubsystem
{
public:
    valor::NeoController NoteHandoffMotor;

    Indexer(frc::TimedRobot *robot);

    ~Indexer();

    void init();

    // void assessInputs();
    // void analyzeDashboard();
    // void assignOutputs();

    // void resetState();
    
    // void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        bool inHandoff;
    } state;

private:

    double handoffSpeed;

};