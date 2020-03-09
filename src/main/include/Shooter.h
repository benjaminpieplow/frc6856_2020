/**
 * Code for managing the shooter, mainly just a PID loop for keeping it at the right speed
 * See Turret.h for the turret/tracking code
 */

#pragma once

#include "ctre/Phoenix.h"
#include <frc/SerialPort.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

class Shooter {
    public:
    Shooter(int CANID);
    ~Shooter();

    //Initiates the turret firing system using the last set RPM
    void EnableShooter();
    //Initiates the turret firing system using the specified RPM
    void EnableShooter(double targetRPM);

    //Stops the shooter
    void DisableShooter();

    //For testing and Development
    void ShooterPower(double power);

    //Returns the number of pixels of the target's lowest pixel from camera bottom
    double GetRawYPixel();

    //Returns a signed ratio of horizontal pixels from center to horizontal pixels
    double GetFOVYFactor();
    double GetFOVYFactor(double rawXPixel);

    //Returns the angle (in degrees) of the target relative to the camera
    double GetFOVYAngle();
    double GetFOVYAngle(double xFactor);

    //Returns the distance of the target from the turret
    double GetTargetDistance();

    //Returns the RPM at the desired target distance
    double GetTargetCalculatedRPM();

    //Returns the current target RPM
    double GetShooterRPM();


    //Is the shooter ready?
    bool ShooterReady();
    //Is the shooter underspeed?
    bool ShooterUnderspeed();
    //Is the shooter overspeed?
    bool ShooterOverspeed();

    //Spins shooter to setpoint RPM, fires when ready
    bool AutoRPM();

    void ToggleAutoRPM(bool enableButton, bool disableButton);


    private:
    //Target RPM
    double mShooterTargetRPM = 100;

    //Shooter Latch for ToggleAutoRPM
    bool mAutoRPMEnabled = false;

    //Sample Rate Multiplier
    const double mSampleRateModifier = 10;

    //Allowed Error when checking speed (Percent)
    const double mAllowedErrorMargin = 5;

    //How many encoder ticks map to one revolution of the accelerator shaft
    const int encoderUnitsPerRevolution = 2048;



    //Camera Resolution
    const double mCameraXRes = 640;
    const double mCameraYRes = 480;
    //Camera FOV
    const double mCameraXFOV = 61;
    const double mCameraYFOV = 34.3;

    //Physical Parameters
    const double mCameraMountAngle = 5.6;      //Degrees
    const double mTurretHeight = 0.87;          //Meters
    const double mTargetBottomHeight = 2.159;   //Meters
    const double mTargetCameraElevation = this->mTargetBottomHeight - this->mTurretHeight;
    
    //Shooter Motor
    WPI_TalonFX* m_pShooterMotor;
    WPI_TalonSRX* m_pFeedMotor;

    //Ultrasonic Sensor
    frc::SerialPort* m_pRangerSerial;

};