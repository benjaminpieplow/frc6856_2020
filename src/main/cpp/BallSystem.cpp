
#include <BallSystem.h>

BallSystem::BallSystem(Elevator* pElevatorPointer, Feeder* pFeeder, Shooter* pShooterPointer, Turret* pTurretPointer, IntakeArm* pIntakeArm)
{
    this->m_pElevator = pElevatorPointer;
    this->m_pFeeder = pFeeder;
    this->m_pShooter = pShooterPointer;
    this->m_pTurret = pTurretPointer;
    this->m_pIntakeArm = pIntakeArm;
}

BallSystem::~BallSystem()
{
}

void BallSystem::StartIntake() {
    
}

void BallSystem::AutoVolley(bool enableButton, bool disableButton) {
    if (disableButton || !mAutoVolleyLatch)
    {   //If operator has stopped machine
        mAutoVolleyLatch = false;           //Don't re-start it
        //Shut off all the things
        this->m_pShooter->DisableShooter();
        this->m_pTurret->SetTurretAngle(0);
        this->m_pElevator->ElevatorStop();
    }

    if (enableButton || mAutoVolleyLatch)
    {   //Latch on
    this->mAutoVolleyLatch = true;
        if (this->m_pTurret->AutoTurret())
        {   //If the turret is up to speed,
            if (this->m_pShooter->AutoRPM())
            {   //If the shooter is up to speed
                if (enableButton)
                {   //If the operator is requesting balls to be fed
                    this->m_pFeeder->FeedForward(); //Feed them
                    this->m_pElevator->ElevatorForward();
                }
                else
                {   //If no ball feed requested
                    this->m_pFeeder->FeedStop();    //Do not feed
                    this->m_pElevator->ElevatorStop();
                }
            }
        }
    }
}