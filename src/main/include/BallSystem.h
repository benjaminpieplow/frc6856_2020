/**
 * Simplified control of the entire ball system, leveraging APIs from the subsystems
 */

//#include <Intake.h>
#include <Elevator.h>
#include <Feeder.h>
#include <Shooter.h>
#include <Turret.h>
#include <IntakeArm.h>
#include <DriverIO.h>

class BallSystem
{
public:
    BallSystem(Elevator* pElevatorPointer,
    Feeder* pFeeder,
    Shooter* pShooterPointer,
    Turret* pTurretPointer,
    IntakeArm* pIntakeArm,
    ControllerInput* pPrimaryController,
    JoystickInput* pSecondaryController);
    ~BallSystem();

    void BallSystemPeriodic();
    
    void AutoVolley(bool enableButton, bool disableButton);
    
private:

    //Pointers to Control objects
    Elevator* m_pElevator;
    Feeder* m_pFeeder;
    Shooter* m_pShooter;
    Turret* m_pTurret;
    IntakeArm* m_pIntakeArm;
    ControllerInput* m_pPrimaryController;
    JoystickInput* m_pSecondaryController;

    bool mAutoTurretEnabled = true;

    //Keep track of whether we're running an AutoVolley
    bool mAutoVolleyLatch = false;
    //Track Active Task
    double taskStartTime = 0;
};
