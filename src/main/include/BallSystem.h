/**
 * Simplified control of the entire ball system, leveraging APIs from the subsystems
 */

//#include <Intake.h>
#include <Elevator.h>
#include <Shooter.h>
#include <Turret.h>

class BallSystem
{
public:
    BallSystem(Elevator* pElevatorPointer, Shooter* pShooterPointer, Turret* pTurretPointer);
    ~BallSystem();

    void StartIntake();

    void Volley();
    
private:

    //Pointers to Control objects
    Elevator* m_pElevator;
    Shooter* m_pShooter;
    Turret* m_pTurret;

    double taskStartTime = 0;
};
