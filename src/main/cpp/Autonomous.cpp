/**
 * Contains code for running fully autonomous functions, called during Autonomous Period
 */

#include <Autonomous.h>

Autonomous::Autonomous(Turret* pTurret, Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Elevator* pElevator)
{
    //Assign pointers to private objects
    this->m_pTurret = pTurret;
    this->m_pShooter = pShooter;
    this->m_pLeftDrive = pLeftDrive;
    this->m_pRightDrive = pRightDrive;
    this->m_pElevator = pElevator;
}

Autonomous::~Autonomous() {
}

void Autonomous::CrudeAutonInit() {
    this->m_pCrudeAuton = new CrudeAuton();
}

/**
 * Probably going to grow into an autonomous testing playground.
 * Current goal: Get off the line after shooter is at RPM
 * 
 * Stage 00 ->  Spin up shooter
 * Stage 01 ->  Feed/Elevator AutoFire for 5 seconds
 * Stage 02 ->  Motion Magic -> to position
 */
void Autonomous::CrudeAutonPeriodic() {
    switch (this->m_pCrudeAuton->GetStage())
    {
    case 0:
        this->m_pCrudeAuton->RunStage00(this->m_pShooter);
        break;
    
    case 1:
        this->m_pCrudeAuton->RunStage01(this->m_pShooter, this->m_pElevator);
        break;
    
    case 2:
        this->m_pCrudeAuton->RunStage02(this->m_pLeftDrive, this->m_pRightDrive);
        break;
    
    default:
        break;
    }
}





CrudeAuton::CrudeAuton()
{
}

CrudeAuton::~CrudeAuton() {
}

int CrudeAuton::GetStage() {
    return this->mProgramStage;
}

void CrudeAuton::RunStage00(Shooter* pShooter) {
    if (pShooter->AutoRPM()) {
        this->mProgramStage = 1;
    }
}

void CrudeAuton::RunStage01(Shooter* pShooter, Elevator* pElevator) {
    if (!this->mStageStarted) {
        this->mStageTimer.Reset();
        this->mStageTimer.Start();
        this->mStageStarted = true;
        if (pShooter->AutoRPM()) {
            pElevator->ElevatorForward();
            pElevator->FeederForward();
        }
    } else if (this->mStageTimer.HasPeriodPassed(5)) {
        this->mStageTimer.Reset();
        this->mStageStarted = false;
        this->mProgramStage = 2;

        pElevator->Stop();
        pShooter->DisableShooter();
    } else {
        if (pShooter->AutoRPM()) {
            pElevator->ElevatorForward();
            pElevator->FeederForward();
        } else {
            pElevator->Stop();
        }
    }
}


void CrudeAuton::RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive) {
    pLeftDrive->SetTargetMotionProfileTarget(1);
    pRightDrive->SetTargetMotionProfileTarget(1);
}