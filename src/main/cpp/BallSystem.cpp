
#include <BallSystem.h>

BallSystem::BallSystem(Elevator* pElevatorPointer, Shooter* pShooterPointer, Turret* pTurretPointer, IntakeArm* pIntakeArm)
{
    this->m_pElevator = pElevatorPointer;
    this->m_pShooter = pShooterPointer;
    this->m_pTurret = pTurretPointer;
    this->m_pIntakeArm = pIntakeArm;
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