//Simple Differential Drive

#pragma once

#include "ctre/Phoenix.h"

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

    void SetPWM(double power);

    void InitVelocityControl();

    void SetTargetVelocity(double targetVel);

    private:
    WPI_TalonSRX* pTalonSRX;
    WPI_VictorSPX* pVictorSPX;
};