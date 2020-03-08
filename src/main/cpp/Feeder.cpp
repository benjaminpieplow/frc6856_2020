/**
 * Code to actuate the feeder mechanism
 */

#include <Feeder.h>


Feeder::Feeder(int CANID)
{
    this->m_pFeederMotor = new WPI_TalonSRX(CANID);

    this->m_pFeederMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

Feeder::~Feeder()
{
}


void Feeder::FeedForward() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, this->mFeedForwardPower);
}

void Feeder::FeedReverse() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, this->mFeedReversePower);
}

void Feeder::FeedStop() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, 0);
}

void Feeder::SetFeederPower(double power) {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, power);
}