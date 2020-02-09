//Simple Differential Drive

#pragma once

#include "ctre/Phoenix.h"

/**
 * TankDrive is largely depricated and no longer maintained in favour of AdvancedDrive, which has additional functionality
 */
class TankDrive {

    public:
    TankDrive();    //Initializes the Motor Controllers

    ~TankDrive();

    void setTankDrivePower(double forwardSpeed, double turnSpeed);

    private:
    WPI_TalonSRX* pTalonSRX[2];
    WPI_VictorSPX* pVictorSPX[2];
};



class AdvancedDrive {
    public:
    AdvancedDrive(int talonCANID, int victorCANID);
    ~AdvancedDrive();
    
    //Set Talons to be basic (ramped) PWM controllers
    void InitSimpleRampedControl();

    //Sets Talons as current-sources
    //DO NOT USE!
    void InitCurrentControl();

    //Set Talons to be fancy smart speed controllers
    void InitVelocityControl();

    //Simple, set output PWM (effectively voltage control)
    void SetPWM(double power);

    //Slightly less simple, lset output current
    //DO NOT USE
    void SetCurrent(double power);

    void SetTargetVelocity(double targetVel);

    void SetTargetMotionProfileTarget(double target);

    private:
    WPI_TalonSRX* pTalonSRX;
    WPI_VictorSPX* pVictorSPX;
};