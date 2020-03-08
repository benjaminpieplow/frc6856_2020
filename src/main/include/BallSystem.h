/**
 * Simplified control of the entire ball system, leveraging APIs from the subsystems
 */

//#include <Intake.h>
#include <Elevator.h>
#include <Shooter.h>
#include <Turret.h>
#include <IntakeArm.h>

class BallSystem
{
public:
    BallSystem(Elevator* pElevatorPointer, Shooter* pShooterPointer, Turret* pTurretPointer, IntakeArm* m_pIntakeArm);
    ~BallSystem();

    void StartIntake();

    void AutoVolley();
    
private:

    //Pointers to Control objects
    Elevator* m_pElevator;
    Shooter* m_pShooter;
    Turret* m_pTurret;
    IntakeArm* m_pIntakeArm;

    double taskStartTime = 0;
};
