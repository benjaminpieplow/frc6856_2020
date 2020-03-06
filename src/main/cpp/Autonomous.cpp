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


/**
 * Uses AutoRPM to check if the shooter is at-speed.
 * Once it is, progress to the next stage
 */
void CrudeAuton::RunStage00(Shooter* pShooter) {
    if (pShooter->AutoRPM()) {
        this->mProgramStage = 1;
    }
}

/**
 * Pushes balls through the intake for 5 seconds
 * Only runns the feed when the shooter is at speed
 * Holds (and verifies) shooter speed using Shooter's AutoRPM
 */
void CrudeAuton::RunStage01(Shooter* pShooter, Elevator* pElevator) {
    //If this is the first run
    if (!this->mStageStarted) {
        //Clear and start the timer
        this->mStageTimer.Reset();
        this->mStageTimer.Start();
        //Flag this stage as started
        this->mStageStarted = true;
        //Start balls moving towards shooter
        pElevator->ElevatorForward();
        //If shooter is ready, give it balls
        if (pShooter->AutoRPM()) {
            pElevator->FeederForward();
        }
    //If this stage has run its course
    } else if (this->mStageTimer.HasPeriodPassed(5)) {
        //Clear the timer for the next use
        this->mStageTimer.Reset();
        //Clear the Stage flag for the next guy
        this->mStageStarted = false;
        //Advance the stage
        this->mProgramStage = 2;

        //Shut down involved mechanisms
        pElevator->Stop();
        pShooter->DisableShooter();
    //If 'cruising' in stage
    } else {
        //Move balls towards shooter
        pElevator->ElevatorForward();
        //If at RPM, put balls in shooter
        if (pShooter->AutoRPM()) {
            pElevator->FeederForward();
        //If not at RPM, no balls for shooter
        } else {
            pElevator->Stop();
        }
    }
}

/**
 * Final stage of CrudeAuton, use Motion Profiles to run 1 meter
 */
void CrudeAuton::RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive) {
    pLeftDrive->SetTargetMotionProfileTarget(1);
    pRightDrive->SetTargetMotionProfileTarget(1);
}