/**
 * Code for managing the shooter, including manual override control and vision-integrated tracking
 * See Shooter.h for the shooter wheel code
 */

#pragma once

#include "ctre/Phoenix.h"
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

    //Returns true when the turret is within allowable shooting error margin
    bool GetTurretLocked();

    //If homed, return true. If not, fires "RunAutoHome"
    bool GetHomed();


    //Servo to an angle in degrees from center (-90 <-> +90)
    void SetTurretAngle(double requestAngle);

    //Runs the turret into the home switch
    void RunAutoHome();



    //For testing (or backup), set the power to the turret motor.
    void SetTurretPower(double power);



    private:
    //Motor that rotates the turret
    WPI_TalonSRX* m_pTurretServo;

    //Last known horizontal pixel of the target (initialize at center)
    double mCameraTargetHPos = this->mCameraXRes / 2;

    //Whether the turret is homed
    bool m_Homed = false;
    //Whether an attempt is being made to change that
    bool m_Homing = false;

//Constants
    //Camera Resolution
    const double mCameraXRes = 640;
    const double mCameraYRes = 480;
    //Camera FOV
    const double mCameraXFOV = 61;
    const double mCameraYFOV = 34.3;

    //Number of encoder ticks per turret degree 
    const double mEncoderTicksPerDegree = ((4096 / 360) * 48 * 33.02) / 2.8575; //Was 2940; is ((ticksperrev / degrees) * gearbox * ringdiameter) / piniondiameter

    //Maximum allowable Error between turret and target in degrees
    const double mAllowableTurretError = 5;

    //Offset between homing limit switch and turret zero (pointing directly backwards) in degrees
    const double mHomeFrameOffset = 100;

    //Calculated Constants
    const double mCameraFOVXResRatio = 2 / this->mCameraXRes;

    

};