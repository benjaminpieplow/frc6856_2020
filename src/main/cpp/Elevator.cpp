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


void Elevator::ElevatorForward() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->mElevatorForwardPower);
}
void Elevator::ElevatorReverse() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->mElevatorReversePower);
}
void Elevator::ElevatorStop() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, 0);
}

void Elevator::SetElevatorPower(double power) {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, power);
}

void Elevator::SmartFeed() {
    if (!mSmartFeedLatch)
    {   //If we have not tripped the current threshold
        if (this->m_pElevatorMotor->GetStatorCurrent() > this->mSmartFeedCurrentThreshold)
        {   //If we are over-current
            this->mSmartFeedTimer.Start();  //Start counting over-current seconds
            this->ElevatorReverse();        //Reverse Elevator
        }
        else
        {   //If we are not over-current
            this->mSmartFeedTimer.Reset();  //Reset the timer
            this->ElevatorForward();        //Continue to feed
        }

        if (this->mSmartFeedTimer.HasPeriodPassed(this->mSmartFeedTriggerTime))
        {   //If we have exceeded the current grace period
            this->mSmartFeedLatch = true;   //Latch into reverse
            this->mSmartFeedTimer.Reset();  //Reset timer for reverse stage
            this->ElevatorReverse();        //Reverse the intake
        }
    }
    else
    {   //If the reverse latch has tripped
        if (this->mSmartFeedTimer.HasPeriodPassed(this->mSmartFeedBackOffTime))
        {   //And it is done tripping
            this->mSmartFeedTimer.Reset();  //Reset timer for normal stage
            this->ElevatorForward();        //Run forward
        } else
        {   //If we are still reversing
            this->ElevatorReverse();        //Back up intake
        }
        
    }
    
    
}