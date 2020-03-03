/**
 * The robot does not have a seperate intake motor, so "intake" calls are made via the elevator class
 * This simply separates the agile code for the intake arm
 */

#include <Intake.h>

Intake::Intake(int armCANID) {
    
    this->m_pIntakeMotor = new WPI_TalonSRX(armCANID);
    
    this->m_pIntakeMotor->ConfigFactoryDefault();
    
    this->m_pIntakeMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

}

/**
 * Checks to see if an intake lower is in progress
 * If not, starts one and starts a timer to stop it
 * If so, continues the maneuver
 */
void Intake::LowerIntake() {

    if (this->mRaiseLatch) {
        this->mRaiseLatch = false;
        this->mActionTimer.Reset();
    }
    
    if (this->mLowerLatch && this->mActionTimer.HasPeriodPassed(this->mIntakeLowerTime)) {
        //If there is an action in progress and it should be stopped
        this->mLowerLatch = false;
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, 0);
    } else if (this->mLowerLatch && !this->mActionTimer.HasPeriodPassed(this->mIntakeLowerTime)) {
        //If there is an action and it should keep going
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakeLowerPower);
    } else {
        //If there is no action in progress, start one
        this->mLowerLatch = true;
        this->mActionTimer.Reset();
        this->mActionTimer.Start();
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakeLowerPower);
    }
}

void Intake::RaiseIntake() {

    if (this->mLowerLatch) {
        this->mLowerLatch = false;
        this->mActionTimer.Reset();
    }
    if (this->mRaiseLatch && this->mActionTimer.HasPeriodPassed(this->mIntakeRaiseTime)) {
        //If there is an action in progress and it should be stopped
        this->mLowerLatch = false;
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakeHoldPower);
    } else if (this->mRaiseLatch && !this->mActionTimer.HasPeriodPassed(this->mIntakeRaiseTime)) {
        //If there is an action and it should keep going
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakeRaisePower);
    } else {
        //If there is no action in progress, start one
        this->mRaiseLatch = true;
        this->mActionTimer.Reset();
        this->mActionTimer.Start();
        this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakeRaisePower);
    }
}

void Intake::IntakePeriodic() {
    if (this->mLowerLatch) {
        this->LowerIntake();
    } else
    if (this->mRaiseLatch) {
        this->RaiseIntake();
    }
}

/**
 * Set intake power manually
 */
void Intake::SetIntakePower(double power) {
    this->m_pIntakeMotor->Set(ControlMode::PercentOutput, power);
}