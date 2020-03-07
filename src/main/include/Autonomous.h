/**
 * Contains all the routines, watchdogs and persistent variables for making the robot do things on its own.
 * The Autonomous class oversees all autonomous operations, and coordinates which class is called, allowing us to program different autonomous modes as other classes
 * without worrying about them interfering with each other.
 * If there is a better way of doing this, Feeney please let me know before I implement it.
 */

#pragma once

#include <Shooter.h>
#include <DriveTrain.h>
#include <Turret.h>
#include <Elevator.h>

#include <frc/Timer.h>

#include <frc/smartdashboard/SmartDashboard.h>

class CrudeAuton
{
public:
    CrudeAuton();
    ~CrudeAuton();

    int GetStage();
    void ResetCrudeAuton();
    void RunStage00(Shooter* pShooter);
    void RunStage01(Shooter* pShooter, Elevator* pElevator);
    void RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive);
private:
    frc::Timer mStageTimer;
    bool mStageStarted = false; //Latch, set true when starting a stage
    int mProgramStage = 0;
};

class Autonomous
{
public:
    Autonomous(Turret* pTurret, Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Elevator* pElevator);
    ~Autonomous();

    /**
     * CrudeAuton is the first auton system written, it,
     *  Spins the shooter up to speed
     *  Pushes balls for 5 seconds (regardless of RPM lock)
     *  Shuts everything off
     *  Moves forward a meter
     */
    void CrudeAutonInit();
    void CrudeAutonPeriodic();

private:

    //Pointers to Robot System Objects
    Shooter* m_pShooter;
    Turret* m_pTurret;
    AdvancedDrive* m_pLeftDrive;
    AdvancedDrive* m_pRightDrive;
    Elevator* m_pElevator;

    //Pointers to Auton Systems
    CrudeAuton* m_pCrudeAuton;
};


