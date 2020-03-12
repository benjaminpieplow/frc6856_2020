
#include <BallSystem.h>

BallSystem::BallSystem(Elevator* pElevatorPointer,
Feeder* pFeeder,
Shooter* pShooterPointer,
Turret* pTurretPointer,
IntakeArm* pIntakeArm,
ControllerInput* pPrimaryController,
JoystickInput* pSecondaryController)
{
    this->m_pElevator = pElevatorPointer;
    this->m_pFeeder = pFeeder;
    this->m_pShooter = pShooterPointer;
    this->m_pTurret = pTurretPointer;
    this->m_pIntakeArm = pIntakeArm;
    this->m_pPrimaryController = pPrimaryController;
    this->m_pSecondaryController = pSecondaryController;
}

BallSystem::~BallSystem()
{
}



void BallSystem::BallSystemPeriodic() {
    /** AutoTurret & Manual Turret*/
    if (this->mAutoTurretEnabled)
    {   //If Turret not in Manual Override 
        this->AutoVolley(this->m_pSecondaryController->getRawButton(1), this->m_pSecondaryController->getRawButton(3));
            
        if (this->m_pPrimaryController->getRTriggerBool())
        {   //If driver wants intake
            this->m_pElevator->ElevatorForward();
        }
        else if (this->m_pPrimaryController->getLTriggerBool())
        {   //If driver wants balls out
            this->m_pElevator->ElevatorReverse();
        }
        else if (!this->m_pSecondaryController->getRawButton(1))
        {   //If operator is not running AutoVolley Feed
            this->m_pElevator->ElevatorStop();
        }
    }
    else
    {   //If Turret is in Manual Override
        //Turret Control
        this->m_pTurret->SetTurretAngle(this->m_pSecondaryController->getJoyX() * 30);

        //Shooter Control
        if (this->m_pSecondaryController->getRawButton(2))
        {   //If operator wants turret spun up
            this->m_pShooter->EnableShooter(4000);
        }
        else if (this->m_pSecondaryController->getRawButton(3))
        {
            this->m_pShooter->DisableShooter();
        }

        //Feeder Control
        if (this->m_pSecondaryController->getRawButton(1))
        {   //If operator wants to feed balls to shooter
            this->m_pFeeder->FeedForward();
        }
        else
        {   //No request, no feed
            this->m_pFeeder->FeedStop();
        }

        //Elevintake Control
        if (this->m_pPrimaryController->getLTriggerBool())
        {   //If driver wants intake reverse
            this->m_pElevator->ElevatorReverse();
        }
        else if (this->m_pSecondaryController->getRawButton(1) || this->m_pPrimaryController->getRTriggerBool())
        {   //If either operator or driver want intake fwd
            this->m_pElevator->ElevatorForward();
        }
        else
        {   //If neither wants either.
            this->m_pElevator->ElevatorStop();
        }
    }   //End AutoTurret Manual Override
    
    
    
    
}

void BallSystem::AutoVolley(bool enableButton, bool disableButton) {
//    if (disableButton || !mAutoVolleyLatch)
    if (disableButton)
    {   //If operator has stopped machine
        mAutoVolleyLatch = false;           //Don't re-start it
        //Shut off all the things
        this->m_pShooter->DisableShooter();
        this->m_pTurret->SetTurretAngle(0);
        this->m_pFeeder->FeedStop();
        this->m_pElevator->ElevatorStop();
        //Note: Elevator may jam in "on" state. Refactor required to fix
    }

    if (enableButton || mAutoVolleyLatch)
    {   //Latch on
    this->mAutoVolleyLatch = true;
        if (this->m_pTurret->AutoTurret())
        {   //If the turret locked,
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
            else
            {   //If the shooter is not at speed
                this->m_pFeeder->FeedStop();    //Do not feed
                this->m_pElevator->ElevatorStop();
            }
        }
        else
        {   //If the turret is not locked
            this->m_pShooter->DisableShooter();
            this->m_pFeeder->FeedStop();    //Do not feed
            this->m_pElevator->ElevatorStop();
        }
        
    }
}