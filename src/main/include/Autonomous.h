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
#include <Feeder.h>
#include <BallSystem.h>

#include <frc/Timer.h>

#include <frc/smartdashboard/SmartDashboard.h>

class BasicAuton
{
public:
    BasicAuton();
    ~BasicAuton();

    int GetStage();
    void ResetBasicAuton(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive);

    void RunStage00(BallSystem* pBallSystem);
    void RunStage01(BallSystem* pBallSystem);
    void RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive);

private:
    frc::Timer mStageTimer;
    bool mStageStarted = false;
    int mProgramStage = 0;

    double stage00Delay = 7;    //Hold for X seconds
    double stage01Delay = 3;    //Change if you want, it takes 2.5 seconds to empty the hopper
    double stage02Distance = -1; //Move X meters forward
};


class CrudeAuton
{
public:
    CrudeAuton();
    ~CrudeAuton();

    int GetStage();
    void ResetCrudeAuton();
    void RunStage00(Shooter* pShooter);
    void RunStage01(Shooter* pShooter, Elevator* pElevator, Feeder* pFeeder);
    void RunStage02(AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive);
private:
    frc::Timer mStageTimer;
    bool mStageStarted = false; //Latch, set true when starting a stage
    int mProgramStage = 0;
};

class Autonomous
{
public:
    Autonomous(Turret* pTurret, Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Elevator* pElevator, Feeder* pFeeder, BallSystem* pBallSystem);
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

    void BasicAutonInit();
    void BasicAutonPeriodic();



private:

    //Pointers to Robot System Objects
    Shooter* m_pShooter;
    Turret* m_pTurret;
    AdvancedDrive* m_pLeftDrive;
    AdvancedDrive* m_pRightDrive;
    Elevator* m_pElevator;
    Feeder* m_pFeeder;
    BallSystem* m_pBallSystem;

    //Pointers to Auton Systems
    CrudeAuton* m_pCrudeAuton;
    BasicAuton* m_pBasicAuton;
};

