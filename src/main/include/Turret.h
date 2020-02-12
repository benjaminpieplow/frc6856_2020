/**
 * Code for managing the shooter, including manual override control and vision-integrated tracking
 * See Shooter.h for the shooter wheel code
 */

#include "ctre/Phoenix.h"

class Turret {
    public:
    Turret(int CANID);
    ~Turret();

    void SetTurretTarget(double position);

    private:
    //Motor that rotates the turret
    WPI_TalonSRX* m_pTurretServo;

};