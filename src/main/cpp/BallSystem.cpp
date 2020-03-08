
#include <BallSystem.h>

BallSystem::BallSystem(Elevator* pElevatorPointer, Shooter* pShooterPointer, Turret* pTurretPointer)
{
    this->m_pElevator = pElevatorPointer;
    this->m_pShooter = pShooterPointer;
    this->m_pTurret = pTurretPointer;
}

BallSystem::~BallSystem()
{
}

void BallSystem::StartIntake() {
    //TODO:
}

void BallSystem::AutoVolley() {

    if (this->m_pTurret->GetTurretLocked()) {
        //Fire when ready!
    }
}