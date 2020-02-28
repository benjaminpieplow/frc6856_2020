/**
 * Contains all the routines, watchdogs and persistent variables for making the robot do things on its own.
 */

#include <Shooter.h>
#include <DriveTrain.h>
#include <Turret.h>

class Autonomous
{
public:
    Autonomous();
    ~Autonomous();

    //Start "Get off the line!"
    void InitCrudeAuton();
    //


    //Start simple autonomous routine to fire the shooter and move off the line
    void InitBasicAuton();
    //Run simple autonomous routine to fire the shooter and move off the line
    void RunBasicAuton(Shooter* pShooter, AdvancedDrive* pLeftDrive, AdvancedDrive* pRightDrive, Turret* pTurret);

private:
    //Keeps track of what is currently to be executed
    int mProgramStage = 0;
};