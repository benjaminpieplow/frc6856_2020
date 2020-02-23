/**
 * Moves balls from intake to turret feed
 * Includes code to push ball into shooter
 */

#include "ctre/Phoenix.h"

class Elevator
{

public:
    Elevator(int elevatorCANID);
    ~Elevator();

    void Forward();
    void Reverse();

private:
    //Elevator Motor
    WPI_TalonFX* m_pElevatorMotor;

    double forwardPower = 0.5;
    
};
