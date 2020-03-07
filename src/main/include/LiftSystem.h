/**
 * Bro-bot, do you even lift?
 * Fairly simple system, go up, go down, don't overrun limt switches
 */

#pragma once

#include <ctre/Phoenix.h>

class LiftSystem
{
public:
    LiftSystem(int CANID);
    ~LiftSystem();

    void LiftRaise();   //Move the mast UP
    void LiftClimb();   //Move the mast DOWN
    void LiftStop();

    void LiftSystemPower(double power);
private:
    //Mast Raise Power
    double liftRaisePower = 0.5;
    double liftLowerPower = -1;
    //Lift System Talon
    WPI_TalonSRX* m_pLiftTalon;
};
