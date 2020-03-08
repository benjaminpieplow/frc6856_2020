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
    void SmartIntake();

    //Reverses for half a second before feeding to turret
    void SmartFeed();

    //Testing
    void SetElevatorPower(double power);

private:
    //Elevator Motor
    WPI_TalonSRX* m_pElevatorMotor;


    //SmartIntake Current Threshold (amps)
    const double mSmartIntakeCurrentThreshold = 20;
    //SmartIntake Trigger Time
    const double mSmartIntakeTriggerTime = 2;
    //SmartIntake BackOff Time
    const double mSmartIntakeBackOffTime = 2;
    //SmartIntake BackOff Latch
    bool mSmartIntakeLatch = false;

    //SmartIntake timer
    frc::Timer mSmartIntakeTimer;


    //SmartFeed Reverse Time
    const double mSmartFeedReverseTime = 0.5;
    //SmartFeed Cooldown Time
    const double mSmartFeedCooldownTime = 2;
    //SmartFeed Latch
    bool mSmartFeedLatch = false;

    //SmartFeed Timer
    frc::Timer mSmartFeedTimer;
    


    
    const double mElevatorForwardPower = 0.75;
    const double mElevatorReversePower = -0.5;
    
};
