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
    valor::NeoController ITBRollerMotor;

    Indexer(frc::TimedRobot *robot);

    ~Indexer();

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    void handoff();
    void rollerIntake();

    double getHandOffSpeed();
    double getRollerSpeed();

    void InitSendable(wpi::SendableBuilder& builder);

    struct x
    {
        bool inHandoff;
        bool isIndexIntake;
    } state;

private:

    double maxHandoffSpeed;
    double maxRollerSpeed;

};