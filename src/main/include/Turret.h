/**
 * Code for managing the shooter, including manual override control and vision-integrated tracking
 * See Shooter.h for the shooter wheel code
 */

#pragma once

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Turret {
    public:
    Turret(int CANID);
    ~Turret();

    //Returns the number of pixels of the target from camera left
    double GetRawXPixel();

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

    //Returns true when a valid target is being tracked
    bool GetTargetTracked();

    //Returns true when the turret is within allowable shooting error margin
    bool GetTurretLocked();

    //If homed, return true. If not, fires "RunAutoHome"
    bool GetHomed();

    //Servo to an angle in degrees from center (-90 <-> +90)
    void SetTurretAngle(double requestAngle);


    //Automatically locks the target, if none found goes zero
    bool AutoTurret();

    //For testing (or backup), set the power to the turret motor.
    void SetTurretPower(double power);



    private:
    //Motor that rotates the turret
    WPI_TalonSRX* m_pTurretServo;

    //Last known horizontal pixel of the target (initialize at center)
    double mCameraTargetHPos = this->mCameraXRes / 2;


    //If we've triggered the Reverse flag
    bool mReverseLimitTripped = false;
    bool mForwardLimitTripped = false;

//BEGIN OLD HOMING CODE
    //Whether the turret is homed
    bool mHomed = false;
    //Whether an attempt is being made to change that
    bool mHoming = false;
    //Whether we're on the home switch, pushing off
    bool mZeroHit = false;

//Constants
    //Camera Resolution
    const double mCameraXRes = 640;
    const double mCameraYRes = 480;
    //Camera FOV
    const double mCameraXFOV = 61;
    const double mCameraYFOV = 34.3;

    //Turret Soft Limit
    const int mSoftLimitFromCenter = 45;

    //Number of encoder ticks per turret degree 
    const double mEncoderTicksPerDegree = ((4096 / 360)  * 33) / 3.3; //Was 2940; is ((ticksperrev / degrees)  * ringdiameter) / piniondiameter

    //Maximum allowable Error between turret and target in degrees
    const double mAllowableTurretError = 2;

    //Offset between homing limit switch and turret zero (pointing directly backwards) in degrees, should be negative
    const double mHomeFrameOffset = 71.54; //Hacky limit switch offset

    //Power with which to home (TODO: Update to velocity)
    const double mHomeVelocity = 30;

    //Calculated Constants
    const double mCameraFOVXResRatio = 2 / this->mCameraXRes;

    

};