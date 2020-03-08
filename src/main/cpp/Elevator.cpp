/**
 * Code for the elevator subsystem.
 * Current plan is to create a class overseeing intake, elevation, feeder, turret and shooter objects, coordinating them
 * Will check with Feeney first
 */

#include <Elevator.h>


Elevator::Elevator(int CANID)
{
    this->m_pElevatorMotor = new WPI_TalonSRX(CANID);

    this->m_pElevatorMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
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