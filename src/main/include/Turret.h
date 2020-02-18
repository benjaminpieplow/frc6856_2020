/**
 * Code for managing the shooter, including manual override control and vision-integrated tracking
 * See Shooter.h for the shooter wheel code
 */

#include "ctre/Phoenix.h"

class Turret {
    public:
    Turret(int CANID);
    ~Turret();

    void SetTurretPower(double power);

    //Returns true when turret is aimed at target
    bool GetLockState();

    //Returns the angle of the target relative to the turret in degrees
    double GetFOVTargetXAngle();
    //Converts from FOVTargetFactor to FOVTargetAngle
    double GetFOVTargetAngle(double FOVTargetFactor);

    //Returns the angle of the target relative to the turret as a factor of the camera's FOV (-1 to +1)
    double GetFOVTargetFactor();
    //Converts from hPos to FOVTargetFactor and sets a new target hPos
    double GetFOVTargetFactor(double hPos);

    //Returns the angle of the target relative to the frame
    double GetFrameTargetAngle();



    private:
    //Motor that rotates the turret
    WPI_TalonSRX* m_pTurretServo;

    //Last known horizontal pixel of the target (initialize at center)
    double pCameraTargetHPos = this->pCameraXRes / 2;
    //Last known horizontal angle of the target
    double pCameraTargetFOVXAngle = 0;
    //Last known horizontal angle factor between camera FOV and target
    double pCameraTargetFOVXFactor = 0;

//Constants
    //Camera Resolution
    const double pCameraXRes = 640;
    const double pCameraYRes = 480;
    //Camera FOV
    const double pCameraXFOV = 61;
    const double pCameraYFOV = 34.3;
    //Encoder to turret ratio
    const double pEncoderTurretRatio = 2940;

    //Calculated Constants
    const double pCameraFOVResRatio = 2 / this->pCameraXRes;

    

};