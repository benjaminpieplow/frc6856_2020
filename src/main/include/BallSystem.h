/**
 * Simplified control of the entire ball system, leveraging APIs from the subsystems
 */

//#include <Intake.h>
#include <Elevator.h>
#include <Feeder.h>
#include <Shooter.h>
#include <Turret.h>
#include <IntakeArm.h>

class BallSystem
{
public:
    BallSystem(Elevator* pElevatorPointer, Feeder* pFeeder, Shooter* pShooterPointer, Turret* pTurretPointer, IntakeArm* m_pIntakeArm);
    ~BallSystem();

    
    void StartIntake();

    void AutoVolley(bool enableButton, bool disableButton);
    
private:

    //Pointers to Control objects
    Elevator* m_pElevator;
    Feeder* m_pFeeder;
    Shooter* m_pShooter;
    Turret* m_pTurret;
    IntakeArm* m_pIntakeArm;

    bool mAutoVolleyLatch = false;

    double taskStartTime = 0;
};
