/**
 * Code for managing the shooter, mainly just a PID loop for keeping it at the right speed
 * See Turret.h for the turret/tracking code
 */

#pragma once

#include "ctre/Phoenix.h"

class Shooter {
    public:
    Shooter(int CANID);
    ~Shooter();

    //How many encoder ticks map to one revolution of the accelerator shaft
    const int encoderUnitsPerRevolution = 1000;

    //Initiates the turret firing system using the last set RPM
    void EnableShooter();

    bool IsTurretReady();

    //Update Target RPM and send to Talon
    void SetTargetVelocity(double targetRPM);

    //Remind Talon what the target RPM is
    void SetTargetVelocity();


    private:
    //Is the mechanism enabled?
    bool mShooterEnabled = false;
    //Is the shooter ready?
    bool mShooterReady = false;
    //Is the shooter underspeed?
    bool mShooterUnderspeed = false;
    //Is the shooter overspeed?
    bool mShooterOverspeed = false;

    //Target RPM
    double mShooterTargetRPM = 3000;
    
    //Shooter Motor
    WPI_TalonSRX* m_pShooterMotor;
};