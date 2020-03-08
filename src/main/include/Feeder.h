/**
 * Moves the ball from the elevator into the shooter
 */

#pragma once

#include <ctre/Phoenix.h>

class Feeder
{
public:
    Feeder(int CANID);
    ~Feeder();

    void FeedForward();
    void FeedReverse();
    void FeedStop();

    void SetFeederPower();

private:
    const double mFeedForwardPower = 1;
    const double mFeedReversePower = -1;

    //Motor Controller
    WPI_TalonSRX* m_pFeederMotor;
};
