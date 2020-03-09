/**
 * Code for managing the shooter, including manual override control and vision-integrated tracking
 * See Shooter.h for the shooter wheel code
 */

#pragma once

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

class Turret {
    public:
    Turret(int CANID);
    ~Turret();

    //Returns the number of pixels of the target from camera left
    double GetRawXPixel();

    //Returns a new pixel every x seconds (set below)
    double GetDelayedXPixel();

    //Returns a signed ratio of horizontal pixels from center to horizontal pixels
    double GetFOVXFactor();
    double GetFOVXFactor(double rawXPixel);

    //Returns the angle (in degrees) of the target relative to the camera
    double GetFOVXAngle();
    double GetFOVXAngle(double xFactor);

    //Returns the current measured orientation of the turret from the frame in degrees
    double GetTurretFrameAngle();

    //Returns the current measured angle of the target from the frame in degrees
    double GetTargetFrameAngle();

    //Returns a tracking-ready dampened TargetFrameAngle
    double GetDampenedTargetFrameAngle();

    //Returns true when a valid target is being tracked
    bool GetTargetTracked();

    //Returns true when the turret is within allowable shooting error margin
    bool GetTurretLocked();

    //If homed, return true. If not, fires "RunAutoHome"
    bool GetHomed();

    //Servo to an angle in degrees from center (-90 <-> +90)
    void SetTurretAngle(double requestAngle);

    //Set a velocity on the turret
    void SetTurretVelocity(double velocity);


    //Automatically locks the target, if none found goes zero
    bool AutoTurret();

    //For testing (or backup), set the power to the turret motor.
    void SetTurretPower(double power);

    void DisableTurret();



    private:
    //Motor that rotates the turret
    WPI_TalonSRX* m_pTurretServo;


    //If we've triggered the Reverse flag
    bool mReverseLimitTripped = false;
    bool mForwardLimitTripped = true;   //Clever hack - Homes turret on startup

    //Whether the turret is homed, mostly for startup
    bool mHomed = false;


//Compensate for LemonLight Delay
    //Timer tracks when to get new vision updates
    frc::Timer mVisionUpdateTimer;    
    //Holds last position
    double mLastXPixel = 0;
    //Update Frequency
    const double mDelayedVisionWaitSeconds = 2;
    //Dampening on vision X angle (effective Kp for vision)
    const double mTargetFrameAngleDamp = 0.2;

//Global Constants
    //Camera Resolution
    const double mCameraXRes = 640;
    const double mCameraYRes = 480;
    //Camera FOV
    const double mCameraXFOV = 61;
    const double mCameraYFOV = 34.3;

    //Turret Soft Limit
    const int mSoftLimitFromCenter = 55;

    //Number of encoder ticks per turret degree 
    const double mEncoderTicksPerDegree = ((4096 / 360) * 33) / 3.2; //((ticksperrev / degrees)  * ringdiameter) / piniondiameter

    //Maximum allowable Error between turret and target in degrees
    const double mAllowableTurretError = 2;

    //Offset between homing limit switch and turret zero (pointing directly backwards) in degrees, should be negative
    const double mHomeFrameOffset = 89.34; //Hacky limit switch offset

    //Power with which to home (TODO: Update to velocity)
    const double mHomeVelocity = 30;

    //Calculated Constants
    const double mCameraFOVXResRatio = 2 / this->mCameraXRes;

    

};