/**
 * Code for the elevator subsystem.
 * Current plan is to create a class overseeing intake, elevation, feeder, turret and shooter objects, coordinating them
 * Will check with Feeney first
 */

#include <Elevator.h>


Elevator::Elevator(int CANID)
{
    this->m_pElevatorMotor = new WPI_TalonSRX(CANID);

    this->m_pElevatorMotor->ConfigFactoryDefault();
    this->m_pElevatorMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, 0);
    this->m_pElevatorMotor->SetInverted(true);

}

Elevator::~Elevator()
{
}

/**
 * Intake Balls and move towards shooter
 */
void Elevator::ElevatorForward() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->mElevatorForwardPower);
}

/**
 * Reject Balls
 */
void Elevator::ElevatorReverse() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->mElevatorReversePower);
}

/**
 * Stop Elevator Motors
 */
void Elevator::ElevatorStop() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, 0);
}

/**
 * Direct Power Control
 */
void Elevator::SetElevatorPower(double power) {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, power);
}


/**
 * Monitor current for prolonged high-current draw
 * If detected, reverse elevator for set back-off time, then continue running forward
 * Jamie is a smart cookie
 */
void Elevator::SmartIntake() {
    if (!mSmartIntakeLatch)
    {   //If we have not tripped the current threshold and,
        if (this->m_pElevatorMotor->GetStatorCurrent() > this->mSmartIntakeCurrentThreshold)
        {   //We are over-current
            this->mSmartIntakeTimer.Start();  //Start counting over-current seconds
            this->ElevatorReverse();        //Reverse Elevator
        }
        else
        {   //We are not over-current
            this->mSmartIntakeTimer.Reset();  //Reset the timer
            this->ElevatorForward();        //Continue to run
        }

        if (this->mSmartIntakeTimer.HasPeriodPassed(this->mSmartIntakeTriggerTime))
        {   //If we have exceeded the current grace period
            this->mSmartIntakeLatch = true;   //Latch into reverse
            this->mSmartIntakeTimer.Reset();  //Reset timer for reverse stage
            this->ElevatorReverse();        //Reverse the intake
        }
    }
    else
    {   //If the reverse latch has tripped
        if (this->mSmartIntakeTimer.HasPeriodPassed(this->mSmartIntakeBackOffTime))
        {   //And it is done tripping
            this->mSmartIntakeTimer.Reset();  //Reset timer for normal stage
            this->ElevatorForward();        //Run forward
            this->mSmartIntakeLatch = false;  //Reset latch
        } else
        {   //If we are still reversing
            this->ElevatorReverse();        //Back up intake
        }
        
    }
    
}

void Elevator::SmartFeed() {
    if (this->mSmartFeedLatch && this->mSmartFeedTimer.HasPeriodPassed(this->mSmartFeedReverseTime))
    {   //SmartFeed BackOff is engaged and finished
        this->mSmartFeedLatch = false;
        this->mSmartFeedTimer.Reset();  //Reset timer for Cooldown Tracking
        this->mSmartFeedTimer.Start();  //Keep timer running
        this->ElevatorForward();
    }
    else if (this->mSmartFeedLatch && !this->mSmartFeedTimer.HasPeriodPassed(this->mSmartFeedReverseTime))
    {   //SmartFeed BackOff is engaged but not finished
        this->mSmartFeedLatch = true;
        this->mSmartFeedTimer.Start();    //Make sure system doesn't infiniloop if paused
        this->ElevatorReverse();
    }
    else if (!this->mSmartFeedLatch && this->mSmartFeedTimer.HasPeriodPassed(this->mSmartFeedCooldownTime))
    {   //If SmartFeed is not engaged and the, engage SmartFeed BackOff
        this->mSmartFeedLatch = true;
        this->mSmartFeedTimer.Reset();
        this->mSmartFeedTimer.Start();
        this->ElevatorReverse();
    }
    else
    {   //If Smartfeed is not engaged and hasn't 
        this->mSmartFeedLatch = false;
        this->mSmartFeedTimer.Start();
    }
    
    
    
}