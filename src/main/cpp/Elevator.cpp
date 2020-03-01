/**
 * Code for the elevator subsystem.
 * Current plan is to create a class overseeing intake, elevation, feeder, turret and shooter objects, coordinating them
 * Will check with Feeney first
 */

#include <Elevator.h>


Elevator::Elevator(int elevatorCANID, int feederCANID)
{
    this->m_pElevatorMotor = new WPI_TalonSRX(elevatorCANID);
    this->m_pFeederMotor = new WPI_TalonSRX(feederCANID);

    this->m_pElevatorMotor->SetInverted(true);
}

Elevator::~Elevator()
{
}


void Elevator::ElevatorForward() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->forwardPower);
}
void Elevator::ElevatorReverse() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, -1);
}
void Elevator::ElevatorStop() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, 0);
}

void Elevator::FeederForward() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, 1);
}
void Elevator::FeederReverse() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, -1);
}
void Elevator::FeederStop() {
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, 0);
}

void Elevator::Stop() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, 0);
    this->m_pFeederMotor->Set(ControlMode::PercentOutput, 0);
}