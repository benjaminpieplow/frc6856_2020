/**
 * Contains code for running fully autonomous functions, called during Autonomous Period
 */

#include <Autonomous.h>

Autonomous::Autonomous(/* args */)
{
}

Autonomous::~Autonomous()
{
}

/**
 * Stage 00 ->  Engage AutoShooter
 * Stage 00 ->  Complete when Shooter at speed
 * Stage 01 ->  Engage AutoTarget
 * Stage 01 ->  Complete when Target Locked
 * Stage 02 ->  Engage Volley
 * Stage 02 ->  Complete when 5 balls fired
 * Stage 03 ->  Engage Motion Profile
 */
void Autonomous::InitCrudeAuton() {
    this->mProgramStage = 0;
}

void Autonomous::RunCrudeAuton(Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Turret* pTurret) {
    switch (this->mProgramStage)
    {
    case 0:
        pShooter->EnableShooter();
        break;

    case 1:
    
    default:
        break;
    }
}