/**
 * Moves balls from intake to turret feed
 */
#pragma once

#include <ctre/Phoenix.h>
#include <frc/Timer.h>

class Elevator {

public:
    Elevator(int CANID);
    ~Elevator();

    //Elevator
    void ElevatorForward();
    void ElevatorReverse();
    void ElevatorStop();

    //Feeds forward, if jam detected, reverses for 0.5 seconds
    void SmartFeed();

    //Testing
    void SetElevatorPower(double power);

private:
    //Elevator Motor
    WPI_TalonSRX* m_pElevatorMotor;

    //SmartFeed Current Threshold (amps)
    double mSmartFeedCurrentThreshold = 40;
    //SmartFeed Trigger Time
    double mSmartFeedTriggerTime = 1;
    //SmartFeed BackOff Time
    double mSmartFeedBackOffTime = 0.75;
    //SmartFeed BackOff Latch
    bool mSmartFeedLatch = false;

    //SmartFeed timer
    frc::Timer mSmartFeedTimer;
    
    const double mElevatorForwardPower = 0.5;
    const double mElevatorReversePower = -0.5;
    
};
