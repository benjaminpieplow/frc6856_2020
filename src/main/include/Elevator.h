/**
 * Moves balls from intake to turret feed
 */
#pragma once

#include "ctre/Phoenix.h"

class Elevator {

public:
    Elevator(int CANID);
    ~Elevator();

    //Elevator
    void ElevatorForward();
    void ElevatorReverse();
    void ElevatorStop();

    //Testing
    void SetElevatorPower(double power);

private:
    //Elevator Motor
    WPI_TalonSRX* m_pElevatorMotor;
    
    const double mElevatorForwardPower = 1;
    const double mElevatorReversePower = -1;
    
};
