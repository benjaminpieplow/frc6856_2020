
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

void BallSystem::RunIntake() {
    //TODO:
}

void BallSystem::Volley() {
    this->m_pShooter->EnableShooter();

    if (this->m_pShooter->ShooterReady()) {
        this->m_pElevator->Forward();
    }
}