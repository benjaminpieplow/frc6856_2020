/**
 * Moves balls from intake to turret feed
 * Includes code to push ball into shooter
 */
#pragma once

#include "ctre/Phoenix.h"

class Elevator {

public:
    Elevator(int elevatorCANID, int feederCANID);
    ~Elevator();

    //Elevator
    void ElevatorForward();
    void ElevatorReverse();
    void ElevatorStop();

    //Feeder
    void FeederForward();
    void FeederReverse();
    void FeederStop();

    //Shared
    void Stop();



private:
    //Elevator Motor
    WPI_TalonSRX* m_pElevatorMotor;
    
    //Feeder Motor
    WPI_TalonSRX* m_pFeederMotor;

    double forwardPower = 1;
    
};
