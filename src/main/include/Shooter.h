/**
 * Code for managing the shooter, mainly just a PID loop for keeping it at the right speed
 * See Turret.h for the turret/tracking code
 */

#pragma once

#include "ctre/Phoenix.h"
#include <frc/SerialPort.h>

class Shooter {
    public:
    Shooter(int CANID);
    ~Shooter();

    //How many encoder ticks map to one revolution of the accelerator shaft
    const int encoderUnitsPerRevolution = 2048;

    //Initiates the turret firing system using the last set RPM
    void EnableShooter();
    void EnableShooter(double targetRPM);
    void EnableAutoShooter();

    //Kills the shooter
    void DisableShooter();

    //For testing and Development
    void ShooterPower(double power);

    //Returns the current target RPM
    double GetShooterRPM();


    //Is the shooter ready?
    bool ShooterReady();
    //Is the shooter underspeed?
    bool ShooterUnderspeed();
    //Is the shooter overspeed?
    bool ShooterOverspeed();


    private:
    //Target RPM
    double mShooterTargetRPM = 500;

    //Sample Rate Multiplier
    const double mSampleRateModifier = 10;

    //Allowed Error when checking speed (Percent)
    const double mAllowedErrorMargin = 0.1;

    //AutoShooter Point-Blank RPM target
    const double mAutoShooterBaselineRPM = 500;
    //AutoShooter Meters per Volt
    const double mAutoShooterMetersPerMiliVolt = 0.1;
    //AutoShooter RPM per meter
    //TODO: Tune
    const double mAutoShooterRPMPerMeter = 100;


    
    //Shooter Motor
    WPI_TalonFX* m_pShooterMotor;

    //Ultrasonic Sensor
    frc::SerialPort* m_pRangerSerial;

};