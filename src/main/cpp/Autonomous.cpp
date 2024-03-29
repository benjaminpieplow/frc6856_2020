/**
 * Contains code for running fully autonomous functions, called during Autonomous Period
 */

#include <Autonomous.h>

/**
 * All robot functions are accessed via pointers, pointers are passed to the Autonomous system during object creation
 * This also creates sub-auton systems, such as auton routines.
 * All auton routines will have objects created, and must be coded to exit cleanly. In the future, we should only create the auton routine we plan to use,
 * but this is being realized the week before competition so it's not getting done in 2020.
 */
Autonomous::Autonomous(Turret* pTurret, Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Elevator* pElevator, Feeder* pFeeder)
{
    //Assign pointers to private objects
    this->m_pTurret = pTurret;
    this->m_pShooter = pShooter;
    this->m_pLeftDrive = pLeftDrive;
    this->m_pRightDrive = pRightDrive;
    this->m_pElevator = pElevator;
    this->m_pFeeder = pFeeder;

    //Do this ONCE
    this->m_pCrudeAuton = new CrudeAuton();
}

Autonomous::~Autonomous() {
}

/**
 * Setup Crude Auton - Fire and Move
 */
void Autonomous::CrudeAutonInit() {
    this->m_pCrudeAuton->ResetCrudeAuton();
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
        frc::SmartDashboard::PutBoolean("DB/LED 1", true);
        break;
    
    case 1:
        this->m_pCrudeAuton->RunStage01(this->m_pShooter, this->m_pElevator, m_pFeeder);
        frc::SmartDashboard::PutBoolean("DB/LED 2", true);
        break;
    
    case 2:
        this->m_pCrudeAuton->RunStage02(this->m_pLeftDrive, this->m_pRightDrive);
        frc::SmartDashboard::PutBoolean("DB/LED 3", true);
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
 * Resets counters and flags pertaining to stage tracking
 */
void CrudeAuton::ResetCrudeAuton() {
    this->mProgramStage = 0;
    this->mStageStarted = false;
}

/**
 * Uses AutoRPM to check if the shooter is at-speed.
 * Once it is, progress to the next stage
 */
void CrudeAuton::RunStage00(Shooter* pShooter) {
    if (pShooter->AutoRPM()) {
//    if (true) {
        this->mProgramStage = 1;
    }
}

/**
 * Pushes balls through the intake for 5 seconds
 * Only runns the feed when the shooter is at speed
 * Holds (and verifies) shooter speed using Shooter's AutoRPM
 * TODO: If elevator jams, move elevatorfeed inside AutoRPM
 */
void CrudeAuton::RunStage01(Shooter* pShooter, Elevator* pElevator, Feeder* pFeeder) {
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
            pFeeder->FeedForward();
        }
    //If this stage has run its course
    } else if (this->mStageTimer.HasPeriodPassed(4)) {
        //Clear the timer for the next use
        this->mStageTimer.Reset();
        //Clear the Stage flag for the next guy
        this->mStageStarted = false;
        //Advance the stage
        this->mProgramStage = 2;

        //Shut down involved mechanisms
        pElevator->ElevatorStop();
        pFeeder->FeedStop();
        pShooter->DisableShooter();
    //If 'cruising' in stage
    } else {
        //Move balls towards shooter
        pElevator->ElevatorForward();
        //If at RPM, put balls in shooter
        if (pShooter->AutoRPM()) {
//        if (true) {
            pFeeder->FeedForward();
        //If not at RPM, no balls for shooter
        } else {
            pFeeder->FeedStop();
        }
    }
}

/**
 * Final stage of CrudeAuton, use Motion Profiles to run 1 meter
 */
void CrudeAuton::RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive) {
    if (!this->mStageStarted) {
        //Zero Encoders and reset drivetrain
        pLeftDrive->ZeroEncoder();
        pRightDrive->ZeroEncoder();
        pLeftDrive->SetTargetMotionProfileTarget(0);
        pRightDrive->SetTargetMotionProfileTarget(0);
        //Mark stage as started
        this->mStageStarted = true;
    } else {
        pLeftDrive->SetTargetMotionProfileTarget(-2);
        pRightDrive->SetTargetMotionProfileTarget(-2);
    }
}