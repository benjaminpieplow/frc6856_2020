/**
 * Code for the elevator subsystem.
 * Current plan is to create a class overseeing intake, elevation, feeder, turret and shooter objects, coordinating them
 * Will check with Feeney first
 */

#include <Elevator.h>


Elevator::Elevator(int elevatorCANID)
{
    this->m_pElevatorMotor = new WPI_TalonFX(elevatorCANID);
}

Elevator::~Elevator()
{
}


void Elevator::Forward() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, this->forwardPower);
}

void Elevator::Reverse() {
    this->m_pElevatorMotor->Set(ControlMode::PercentOutput, -1);
}